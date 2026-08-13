#include <Arduino.h>
#include <Wire.h>

#define MAG_ADDR 0x0C
#define NUM_IMUS 3
const int CALIBRATION_SAMPLES = 2000;
const unsigned long LOG_DURATION_MS = 100000; 

TwoWire I2Cbus2 = TwoWire(1);

// Per-sensor state
struct IMUState {
  TwoWire* wire;
  uint8_t  mpuAddr;
  bool     sharesBus; // true = needs aux-master mag passthrough (Bus 1 sensors)

  float RateRoll, RatePitch, RateYaw;
  float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
  float AccX, AccY, AccZ;
  float AngleRoll, AnglePitch;
  float AngleRollOffset, AnglePitchOffset;

  float MagX, MagY, MagZ;
  float MagASAx, MagASAy, MagASAz;
  float Heading, HeadingOffset;

  float KalmanAngleRoll, KalmanUncertaintyAngleRoll;
  float KalmanAnglePitch, KalmanUncertaintyAnglePitch;
  float KalmanAngleYaw, KalmanUncertaintyAngleYaw;
};

IMUState imu[NUM_IMUS];
float Kalman1DOutput[] = {0, 0};

uint32_t LoopTimer;
bool loggingActive = false;
unsigned long loggingStartMillis = 0;

const char* CSV_HEADER =
  "Time_s,"
  "AccX_g_S0,AccY_g_S0,AccZ_g_S0,GyroX_dps_S0,GyroY_dps_S0,GyroZ_dps_S0,MagX_uT_S0,MagY_uT_S0,MagZ_uT_S0,KalmanRoll_deg_S0,KalmanPitch_deg_S0,KalmanYaw_deg_S0,"
  "AccX_g_S1,AccY_g_S1,AccZ_g_S1,GyroX_dps_S1,GyroY_dps_S1,GyroZ_dps_S1,MagX_uT_S1,MagY_uT_S1,MagZ_uT_S1,KalmanRoll_deg_S1,KalmanPitch_deg_S1,KalmanYaw_deg_S1,"
  "AccX_g_S2,AccY_g_S2,AccZ_g_S2,GyroX_dps_S2,GyroY_dps_S2,GyroZ_dps_S2,MagX_uT_S2,MagY_uT_S2,MagZ_uT_S2,KalmanRoll_deg_S2,KalmanPitch_deg_S2,KalmanYaw_deg_S2";

// Forward declarations
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
void configure_mpu_once(IMUState &s);
void gyro_signals(IMUState &s);
void mpu_enable_bypass(IMUState &s);
void mpu_switch_to_auxmaster(IMUState &s);
void mag_init_via_bypass(IMUState &s);
bool mag_signals_bypass(IMUState &s);
bool mag_signals_auxmaster(IMUState &s);
float compute_heading(IMUState &s);
float compute_heading_raw(IMUState &s);
float wrap180(float angle);
void start_log_session(void);
void stop_log_session(void);

// 1D Kalman filter — same structure as your single-sensor version,
// reused for every axis of every sensor.
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// One-time register config (LPF, accel/gyro range) — moved OUT of the
// per-loop read function since with 3 sensors, re-writing these every
// single loop iteration triples the wasted I2C overhead for no reason.
void configure_mpu_once(IMUState &s) {
  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x1A); s.wire->write(0x05); // DLPF config
  s.wire->endTransmission();

  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x1C); s.wire->write(0x10); // Accel +-8g
  s.wire->endTransmission();

  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x1B); s.wire->write(0x08); // Gyro +-500 dps
  s.wire->endTransmission();
}

// Read accel + gyro 
void gyro_signals(IMUState &s) {
  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x3B);
  s.wire->endTransmission();
  s.wire->requestFrom(s.mpuAddr, (uint8_t)6);
  int16_t AccXLSB = s.wire->read() << 8 | s.wire->read();
  int16_t AccYLSB = s.wire->read() << 8 | s.wire->read();
  int16_t AccZLSB = s.wire->read() << 8 | s.wire->read();

  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x43);
  s.wire->endTransmission();
  s.wire->requestFrom(s.mpuAddr, (uint8_t)6);
  int16_t GyroX = s.wire->read() << 8 | s.wire->read();
  int16_t GyroY = s.wire->read() << 8 | s.wire->read();
  int16_t GyroZ = s.wire->read() << 8 | s.wire->read();

  s.RateRoll  = (float)GyroX / 65.5;
  s.RatePitch = -(float)GyroY / 65.5;
  s.RateYaw   = -(float)GyroZ / 65.5;

  s.AccX = (float)AccXLSB / 4096;
  s.AccY = (float)AccYLSB / 4096;
  s.AccZ = (float)AccZLSB / 4096;

  s.AngleRoll  = atan(s.AccY / sqrt(s.AccX * s.AccX + s.AccZ * s.AccZ)) * 1 / (3.142 / 180);
  s.AnglePitch = atan(s.AccX / sqrt(s.AccY * s.AccY + s.AccZ * s.AccZ)) * 1 / (3.142 / 180);
}

// Temporarily enable bypass (used during setup/calibration for every
// sensor, and permanently for Sensor C which doesn't share its bus)
void mpu_enable_bypass(IMUState &s) {
  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x6A); s.wire->write(0x00); 
  s.wire->endTransmission();

  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x37); s.wire->write(0x02); 
  s.wire->endTransmission();
}

// Switch a bus-sharing sensor from bypass mode to auxiliary-master
// passthrough: disables bypass, enables the MPU's internal I2C master,
// and configures SLV0 to auto-read AK8963 (7 bytes from HXL) into this
// MPU's own EXT_SENS_DATA00-06 registers every sample.
void mpu_switch_to_auxmaster(IMUState &s) {
  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x37); s.wire->write(0x00); // INT_PIN_CFG: BYPASS_EN = 0
  s.wire->endTransmission();

  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x6A); s.wire->write(0x20); // USER_CTRL: I2C_MST_EN = 1
  s.wire->endTransmission();

  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x24); s.wire->write(0x4D); // I2C_MST_CTRL: wait-for-ES + 400kHz
  s.wire->endTransmission();
  delay(10);

  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x25); s.wire->write(0x8C); // I2C_SLV0_ADDR = 0x0C | read bit
  s.wire->endTransmission();

  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x26); s.wire->write(0x03); // I2C_SLV0_REG = AK8963 HXL
  s.wire->endTransmission();

  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x27); s.wire->write(0x87); // I2C_SLV0_CTRL = enable, length 7
  s.wire->endTransmission();
  delay(10);
}

// Init AK8963 via bypass (ASA read + continuous mode). Safe to call for
// every sensor during setup since each sensor's bypass is exclusively
// active one at a time (setup runs sequentially, not simultaneously).
void mag_init_via_bypass(IMUState &s) {
  s.wire->beginTransmission(MAG_ADDR);
  s.wire->write(0x0A); s.wire->write(0x00); // power down
  s.wire->endTransmission();
  delay(10);

  s.wire->beginTransmission(MAG_ADDR);
  s.wire->write(0x0A); s.wire->write(0x0F); // fuse ROM access mode
  s.wire->endTransmission();
  delay(10);

  s.wire->beginTransmission(MAG_ADDR);
  s.wire->write(0x10);
  s.wire->endTransmission();
  s.wire->requestFrom(MAG_ADDR, (uint8_t)3);
  uint8_t asax = s.wire->read();
  uint8_t asay = s.wire->read();
  uint8_t asaz = s.wire->read();
  s.MagASAx = ((float)(asax - 128) * 0.5 / 128.0) + 1.0;
  s.MagASAy = ((float)(asay - 128) * 0.5 / 128.0) + 1.0;
  s.MagASAz = ((float)(asaz - 128) * 0.5 / 128.0) + 1.0;

  s.wire->beginTransmission(MAG_ADDR);
  s.wire->write(0x0A); s.wire->write(0x00); // power down
  s.wire->endTransmission();
  delay(10);

  s.wire->beginTransmission(MAG_ADDR);
  s.wire->write(0x0A); s.wire->write(0x16); // continuous mode 2, 16-bit
  s.wire->endTransmission();
  delay(10);
}

// Magnetometer read — direct bypass version (Sensor C, alone on its bus)
bool mag_signals_bypass(IMUState &s) {
  s.wire->beginTransmission(MAG_ADDR);
  s.wire->write(0x02); // ST1
  s.wire->endTransmission();
  s.wire->requestFrom(MAG_ADDR, (uint8_t)1);
  uint8_t st1 = s.wire->read();
  if (!(st1 & 0x01)) return false;

  s.wire->beginTransmission(MAG_ADDR);
  s.wire->write(0x03); // HXL
  s.wire->endTransmission();
  s.wire->requestFrom(MAG_ADDR, (uint8_t)7);
  int16_t mx = s.wire->read() | (s.wire->read() << 8);
  int16_t my = s.wire->read() | (s.wire->read() << 8);
  int16_t mz = s.wire->read() | (s.wire->read() << 8);
  uint8_t st2 = s.wire->read();
  if (st2 & 0x08) return false;

  const float scale = 4912.0 / 32760.0;
  float mxc = mx * scale * s.MagASAx;
  float myc = my * scale * s.MagASAy;
  float mzc = mz * scale * s.MagASAz;

  s.MagX =  myc;
  s.MagY =  mxc;
  s.MagZ = -mzc;
  return true;
}

// Magnetometer read — aux-master passthrough version (Sensors A & B,
// sharing Bus 1). Reads EXT_SENS_DATA00-06 from the MPU's OWN address —
// never touches external 0x0C, so no collision between A and B.
bool mag_signals_auxmaster(IMUState &s) {
  s.wire->beginTransmission(s.mpuAddr);
  s.wire->write(0x49); // EXT_SENS_DATA_00
  s.wire->endTransmission();
  s.wire->requestFrom(s.mpuAddr, (uint8_t)7);
  int16_t mx = s.wire->read() | (s.wire->read() << 8);
  int16_t my = s.wire->read() | (s.wire->read() << 8);
  int16_t mz = s.wire->read() | (s.wire->read() << 8);
  uint8_t st2 = s.wire->read();
  if (st2 & 0x08) return false; // overflow flag from AK8963's ST2, relayed through

  const float scale = 4912.0 / 32760.0;
  float mxc = mx * scale * s.MagASAx;
  float myc = my * scale * s.MagASAy;
  float mzc = mz * scale * s.MagASAz;

  s.MagX =  myc;
  s.MagY =  mxc;
  s.MagZ = -mzc;
  return true; 
}

// Tilt-compensated heading (positive = turning right)
float compute_heading(IMUState &s) {
  float rollRad  = s.KalmanAngleRoll  * 3.14159265 / 180.0;
  float pitchRad = s.KalmanAnglePitch * 3.14159265 / 180.0;
  float Xh = s.MagX * cos(pitchRad) + s.MagZ * sin(pitchRad);
  float Yh = s.MagX * sin(rollRad) * sin(pitchRad) + s.MagY * cos(rollRad) - s.MagZ * sin(rollRad) * cos(pitchRad);
  return -atan2(Yh, Xh) * 180.0 / 3.14159265;
}

float compute_heading_raw(IMUState &s) {
  float rollRad  = s.AngleRoll  * 3.14159265 / 180.0;
  float pitchRad = s.AnglePitch * 3.14159265 / 180.0;
  float Xh = s.MagX * cos(pitchRad) + s.MagZ * sin(pitchRad);
  float Yh = s.MagX * sin(rollRad) * sin(pitchRad) + s.MagY * cos(rollRad) - s.MagZ * sin(rollRad) * cos(pitchRad);
  return -atan2(Yh, Xh) * 180.0 / 3.14159265;
}

float wrap180(float angle) {
  while (angle > 180)  angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

// Logging session protocol 
void start_log_session(void) {
  Serial.println("START_LOG");
  Serial.println(CSV_HEADER);
  loggingStartMillis = millis();
  loggingActive = true;
}

void stop_log_session(void) {
  Serial.println("STOP_LOG");
  loggingActive = false;
}

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); 

  Wire.begin(21, 22);       // Bus 1: Sensor A (0x68) + Sensor B (0x69)
  Wire.setClock(400000);
  Wire.setTimeOut(1000);

  I2Cbus2.begin(16, 17);    // Bus 2: Sensor C (alone)
  I2Cbus2.setClock(400000);
  I2Cbus2.setTimeOut(1000);
  delay(250);

  // Assign each sensor's bus, address, and bus-sharing status
  imu[0] = {}; imu[0].wire = &Wire;     imu[0].mpuAddr = 0x68; imu[0].sharesBus = true;  // Sensor A
  imu[1] = {}; imu[1].wire = &Wire;     imu[1].mpuAddr = 0x69; imu[1].sharesBus = true;  // Sensor B
  imu[2] = {}; imu[2].wire = &I2Cbus2;  imu[2].mpuAddr = 0x68; imu[2].sharesBus = false; // Sensor C

  for (int i = 0; i < NUM_IMUS; i++) {
    IMUState &s = imu[i];

    s.KalmanUncertaintyAngleRoll = 2 * 2;
    s.KalmanUncertaintyAnglePitch = 2 * 2;
    s.KalmanUncertaintyAngleYaw = 2 * 2;

    // Wake up
    s.wire->beginTransmission(s.mpuAddr);
    s.wire->write(0x6B); s.wire->write(0x00);
    s.wire->endTransmission();

    configure_mpu_once(s);

    // Temporarily enable bypass to configure the magnetometer — safe
    // even for bus-sharing sensors since setup runs one sensor at a time.
    mpu_enable_bypass(s);
    mag_init_via_bypass(s);

    // Calibration (2000 samples), using bypass-mode mag reads
    float headingSum = 0;
    int headingSamples = 0;

    for (int k = 0; k < CALIBRATION_SAMPLES; k++) {
      gyro_signals(s);
      s.RateCalibrationRoll  += s.RateRoll;
      s.RateCalibrationPitch += s.RatePitch;
      s.RateCalibrationYaw   += s.RateYaw;
      s.AngleRollOffset  += s.AngleRoll;
      s.AnglePitchOffset += s.AnglePitch;

      if (mag_signals_bypass(s)) {
        headingSum += compute_heading_raw(s);
        headingSamples++;
      }
      delay(1);
    }

    s.RateCalibrationRoll  /= CALIBRATION_SAMPLES;
    s.RateCalibrationPitch /= CALIBRATION_SAMPLES;
    s.RateCalibrationYaw   /= CALIBRATION_SAMPLES;
    s.AngleRollOffset      /= CALIBRATION_SAMPLES;
    s.AnglePitchOffset     /= CALIBRATION_SAMPLES;
    if (headingSamples > 0) s.HeadingOffset = headingSum / headingSamples;

    // Switch bus-sharing sensors to aux-master passthrough
    if (s.sharesBus) {
      mpu_switch_to_auxmaster(s);
    }
    // Sensor C (sharesBus == false) simply stays in bypass mode.
  }

  digitalWrite(13, LOW); // all 3 sensors calibrated

  start_log_session();
  LoopTimer = micros();
}

void loop() {
  for (int i = 0; i < NUM_IMUS; i++) {
    IMUState &s = imu[i];

    gyro_signals(s);
    bool magOk = s.sharesBus ? mag_signals_auxmaster(s) : mag_signals_bypass(s);

    s.RateRoll   -= s.RateCalibrationRoll;
    s.RatePitch  -= s.RateCalibrationPitch;
    s.RateYaw    -= s.RateCalibrationYaw;
    s.AngleRoll  -= s.AngleRollOffset;
    s.AnglePitch -= s.AnglePitchOffset;

    kalman_1d(s.KalmanAngleRoll, s.KalmanUncertaintyAngleRoll, s.RateRoll, s.AngleRoll);
    s.KalmanAngleRoll = Kalman1DOutput[0];
    s.KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

    kalman_1d(s.KalmanAnglePitch, s.KalmanUncertaintyAnglePitch, s.RatePitch, s.AnglePitch);
    s.KalmanAnglePitch = Kalman1DOutput[0];
    s.KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    if (magOk) {
      s.Heading = wrap180(compute_heading(s) - s.HeadingOffset);
      float measurementForFilter = s.KalmanAngleYaw + wrap180(s.Heading - s.KalmanAngleYaw);
      kalman_1d(s.KalmanAngleYaw, s.KalmanUncertaintyAngleYaw, s.RateYaw, measurementForFilter);
      s.KalmanAngleYaw = wrap180(Kalman1DOutput[0]);
      s.KalmanUncertaintyAngleYaw = Kalman1DOutput[1];
    } else {
      s.KalmanAngleYaw = wrap180(s.KalmanAngleYaw + s.RateYaw * 0.004);
    }
  }

  float timeStamp = millis() / 1000.0;
  String row = String(timeStamp, 3);
  for (int i = 0; i < NUM_IMUS; i++) {
    IMUState &s = imu[i];
    row += "," + String(s.AccX, 4) + "," + String(s.AccY, 4) + "," + String(s.AccZ, 4) + "," +
           String(s.RateRoll, 4) + "," + String(s.RatePitch, 4) + "," + String(s.RateYaw, 4) + "," +
           String(s.MagX, 3) + "," + String(s.MagY, 3) + "," + String(s.MagZ, 3) + "," +
           String(s.KalmanAngleRoll, 3) + "," + String(s.KalmanAnglePitch, 3) + "," + String(s.KalmanAngleYaw, 3);
  }

  if (loggingActive) {
    Serial.println(row);
    if (millis() - loggingStartMillis >= LOG_DURATION_MS) {
      stop_log_session();
    }
  }

  if (Serial.available()) {
    char cmd = Serial.read();
    if ((cmd == 'r' || cmd == 'R') && !loggingActive) {
      start_log_session();
    } else if ((cmd == 's' || cmd == 'S') && loggingActive) {
      stop_log_session();
    }
  }

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
