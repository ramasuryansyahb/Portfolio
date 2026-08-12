#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/as5600/MagneticSensorAS5600.h"
#include "ekf_bldc.h"

BLDCMotor motor = BLDCMotor(11);
BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense current_sense = LowsideCurrentSense(
    0.003f, -64.0f / 7.0f,
    A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

EKF_BLDC ekf;

// ── AS5600 —─────────────────────────────────────────────
MagneticSensorAS5600 sensor_as5600;

// ── Open-loop parameters ──────────────────────────────────────────────────────
#define OPENLOOP_VELOCITY      3.0f
#define OL_MIN_DURATION_US     500L

#define OL_VOLTAGE_START       1.0f
#define OL_VOLTAGE_TARGET      1.5f
#define OL_VOLTAGE_RAMP_MS     200

// ── Dynamic OL→CL switch conditions ──────────────────────────────────────────
#define BEMF_SWITCH_THRESHOLD  0.25f
#define PK_THETA_CONVERGED     0.010f
#define BEMF_STABLE_CYCLES     150

// ── Closed-loop velocity ramp after switch ────────────────────────────────────
#define FINAL_TARGET_VELOCITY  2.0f
#define RAMP_DURATION_US       1000000L

// ── Logging ───────────────────────────────────────────────────────────────────
#define LOGGING_DURATION_US    20000000L

// ── Runtime state ─────────────────────────────────────────────────────────────
float         target_velocity     = FINAL_TARGET_VELOCITY;
bool          closed_loop_active  = false;
bool          is_logging          = false;
unsigned long openloop_start_us   = 0;
unsigned long previous_ekf_micros = 0;

int           bemf_stable_count   = 0;
float         ol_exit_velocity    = 0.0f;
unsigned long cl_start_us         = 0;

PhaseCurrent_s last_currents = {0.0f, 0.0f, 0.0f};

// ─────────────────────────────────────────────────────────────────────────────
float readEKFAngle() {
    static float last_theta     = 0.0f;
    static float full_rotations = 0.0f;
    static bool  was_open_loop  = true;

    if (!closed_loop_active) {
        was_open_loop = true;
        return 0.0f;
    }

    if (was_open_loop) {
        last_theta     = ekf.getAngle();
        full_rotations = 0.0f;
        was_open_loop  = false;
    }

    float theta = ekf.getAngle();

    if      (theta - last_theta < -_PI) full_rotations += _2PI;
    else if (theta - last_theta >  _PI) full_rotations -= _2PI;

    last_theta = theta;
    return (theta + full_rotations) / 11.0f;
}

void initEKFSensor() {}

GenericSensor sensor_ekf = GenericSensor(readEKFAngle, initEKFSensor);

// ─────────────────────────────────────────────────────────────────────────────
void configureOpenLoop() {
    motor.torque_controller   = TorqueControlType::voltage;
    motor.controller          = MotionControlType::velocity_openloop;
    motor.foc_modulation      = FOCModulationType::SpaceVectorPWM;
    motor.voltage_limit       = OL_VOLTAGE_START;
}

void configureClosedLoop() {
    motor.torque_controller   = TorqueControlType::foc_current;
    motor.controller          = MotionControlType::velocity;
    motor.foc_modulation      = FOCModulationType::SpaceVectorPWM;
    motor.modulation_centered = false;

    motor.PID_current_q.P  = 1.0f;
    motor.PID_current_q.I  = 2.0f;
    motor.LPF_current_q.Tf = 0.003f;
    motor.PID_current_d.P  = 1.0f;
    motor.PID_current_d.I  = 2.0f;
    motor.LPF_current_d.Tf = 0.003f;

    motor.PID_velocity.P           = 0.3f;
    motor.PID_velocity.I           = 1.0f;
    motor.PID_velocity.D           = 0.0f;
    motor.PID_velocity.output_ramp = 300.0f;
    motor.LPF_velocity.Tf          = 0.05f;

    motor.updateCurrentLimit(9.0f);
    motor.updateVoltageLimit(8.0f);

    motor.PID_current_q.reset();
    motor.PID_current_d.reset();
    motor.PID_velocity.reset();
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(1000000);
    delay(3000);

    driver.voltage_power_supply = 12.0f;
    driver.voltage_limit        = 12.0f;
    driver.pwm_frequency        = 25000;
    if (!driver.init()) return;
    motor.linkDriver(&driver);

    current_sense.linkDriver(&driver);
    if (!current_sense.init()) return;
    motor.linkCurrentSense(&current_sense);

    sensor_ekf.init();
    motor.linkSensor(&sensor_ekf);

    // ── AS5600 init —─────────────────────
        sensor_as5600.init();

    motor.phase_resistance    = 0.285f;
    motor.phase_inductance    = 0.00004139f;
    motor.sensor_direction    = Direction::CW;
    motor.zero_electric_angle = 0.0f;

    motor.voltage_limit = 2.0f;
    motor.controller    = MotionControlType::angle_openloop;

    motor.init();
    motor.initFOC();

    // ── Stage 1: Rotor alignment ──────────────────────────────────────────
    unsigned long align_start = millis();
    while (millis() - align_start < 1500) {
        motor.move(0.0f);
        motor.loopFOC();
    }

    // ── Stage 2: OL soft-start voltage ramp ──────────────────────────────
    configureOpenLoop();
    motor.target = OPENLOOP_VELOCITY;
    {
        unsigned long ramp_start = millis();
        while (millis() - ramp_start < (unsigned long)OL_VOLTAGE_RAMP_MS) {
            float alpha = (float)(millis() - ramp_start) / (float)OL_VOLTAGE_RAMP_MS;
            motor.voltage_limit = OL_VOLTAGE_START
                                + alpha * (OL_VOLTAGE_TARGET - OL_VOLTAGE_START);
            motor.loopFOC();
            motor.move(OPENLOOP_VELOCITY);
        }
        motor.voltage_limit = OL_VOLTAGE_TARGET;
    }

    // ── Stage 2b: settle then seed EKF ───────────────────────────────────
    {
        unsigned long settle = millis();
        while (millis() - settle < 10) {
            motor.loopFOC();
            motor.move(OPENLOOP_VELOCITY);
        }
    }
    {
        PhaseCurrent_s c0 = current_sense.getPhaseCurrents();
        ekf.resetToAligned(0.0f);
        ekf.x[0] = c0.a;
        ekf.x[1] = 0.57735026919f * (c0.a + 2.0f * c0.b);
        ekf.constrainOmega(OPENLOOP_VELOCITY * 11.0f);
    }

    previous_ekf_micros = _micros();
    openloop_start_us   = _micros();

    
#define PRINT_EKF_PARAMS 1
#if PRINT_EKF_PARAMS
    Serial.println(F("# ── EKF parameter check ──"));
    Serial.print(F("# J       = ")); Serial.println(ekf.J, 8);
    Serial.print(F("# F_fric  = ")); Serial.println(ekf.F_fric, 8);
    Serial.print(F("# ke      = ")); Serial.println(ekf.ke, 6);
    Serial.print(F("# Q[0..3] = "));
    Serial.print(ekf.Q[0], 6); Serial.print(F(", "));
    Serial.print(ekf.Q[1], 6); Serial.print(F(", "));
    Serial.print(ekf.Q[2], 6); Serial.print(F(", "));
    Serial.println(ekf.Q[3], 6);
    Serial.print(F("# R[0..1] = "));
    Serial.print(ekf.R[0], 6); Serial.print(F(", "));
    Serial.println(ekf.R[1], 6);
    Serial.println(F("# ─────────────────────────"));
#endif

    is_logging = true;
    Serial.println(F("START_LOG"));
    Serial.println(F("Timestamp_s,Omega_ref,Ia_meas,Ib_meas,Ic_meas,Ialpha_meas,Ibeta_meas,Id_meas,Iq_meas,Theta_OL,Theta_EKF,Omega_EKF,Omega_motor,Ialpha_pred,Ibeta_pred,Innov_alpha,Innov_beta,Pk_omega,Pk_theta,Theta_enc,Omega_enc"));
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = _micros();

    if (is_logging && (now - openloop_start_us >= LOGGING_DURATION_US)) {
        is_logging = false;
        Serial.println(F("STOP_LOG"));
    }

    // ── EKF update at 2 kHz ──────────────────────────────────────────────
    if (now - previous_ekf_micros >= 500) {
        float dt_ekf = (float)(now - previous_ekf_micros) / 1e6f;

        PhaseCurrent_s currents = current_sense.getPhaseCurrents();
        last_currents = currents;   
        float I_alpha = currents.a;
        float I_beta  = 0.57735026919f * (currents.a + 2.0f * currents.b);

        ekf.update(motor.Ualpha, motor.Ubeta, I_alpha, I_beta, dt_ekf);

        if (!closed_loop_active) {
            ekf.constrainOmega(OPENLOOP_VELOCITY * 11.0f);
        }

        sensor_as5600.update();

        previous_ekf_micros = now;
    }

    // ── Stage 3: BEMF-based dynamic OL→CL switch ─────────────────────────
    if (!closed_loop_active) {
        bool time_ok = (now - openloop_start_us >= OL_MIN_DURATION_US);
        bool bemf_ok = (ekf.ke * fabsf(ekf.getVelocity()) > BEMF_SWITCH_THRESHOLD);
        bool pk_ok   = (ekf.getPk(3, 3) < PK_THETA_CONVERGED);

        if (time_ok && bemf_ok && pk_ok) {
            if (++bemf_stable_count >= BEMF_STABLE_CYCLES) {
                ekf.x[3]      = _normalizeAngle(motor.electrical_angle);
                ekf.Pk[3*4+3] = 0.05f;

                configureClosedLoop();
                closed_loop_active = true;
                ol_exit_velocity   = ekf.getVelocity() / 11.0f;
                target_velocity    = ol_exit_velocity;
                cl_start_us        = now;
            }
        } else {
            bemf_stable_count = 0;
        }
    }

    motor.loopFOC();

    // ── Motion command ────────────────────────────────────────────────────
    if (!closed_loop_active) {
        motor.move(OPENLOOP_VELOCITY);
    } else {
        float elapsed = (float)(now - cl_start_us);
        if (elapsed < (float)RAMP_DURATION_US) {
            float alpha     = elapsed / (float)RAMP_DURATION_US;
            target_velocity = ol_exit_velocity
                            + alpha * (FINAL_TARGET_VELOCITY - ol_exit_velocity);
        } else {
            target_velocity = FINAL_TARGET_VELOCITY;
        }
        motor.move(target_velocity);
    }

    // ── Serial logging at 500 Hz ─────────────────────────────────────────
    static unsigned long previous_log_us = 0;
    if (is_logging && (now - previous_log_us >= 2000)) {
        previous_log_us = now;

        PhaseCurrent_s currents = last_currents;
        float I_alpha   = currents.a;
        float I_beta    = 0.57735026919f * (currents.a + 2.0f * currents.b);
        float theta_ekf = ekf.getAngle();
        float omega_ekf = ekf.getVelocity();
        float angle_el  = _normalizeAngle(motor.electrical_angle);
        float omega_el  = motor.shaft_velocity * 11.0f;

        float c   = _cos(theta_ekf);
        float s   = _sin(theta_ekf);
        float I_d = (I_alpha * c) + (I_beta * s);
        float I_q = (I_beta  * c) - (I_alpha * s);

        float Ia_pred  = ekf.getPredictedIalpha();
        float Ib_pred  = ekf.getPredictedIbeta();
        float Innov_a  = ekf.getInnovAlpha();
        float Innov_b  = ekf.getInnovBeta();
        float Pk_omega = ekf.getPk(2, 2);
        float Pk_theta = ekf.getPk(3, 3);

        // ── AS5600 passive readings — scaled to electrical units (×P) ──────────
        float theta_enc = _normalizeAngle(sensor_as5600.getAngle() * 11.0f);
        float omega_enc = sensor_as5600.getVelocity() * 11.0f;

        Serial.print((float)(now - openloop_start_us) / 1e6f, 4); Serial.print(',');
        Serial.print(motor.target, 2);      Serial.print(',');
        Serial.print(currents.a, 2);        Serial.print(',');
        Serial.print(currents.b, 2);        Serial.print(',');
        Serial.print(currents.c, 2);        Serial.print(',');
        Serial.print(I_alpha, 2);           Serial.print(',');
        Serial.print(I_beta, 2);            Serial.print(',');
        Serial.print(I_d, 2);               Serial.print(',');
        Serial.print(I_q, 2);               Serial.print(',');
        Serial.print(angle_el, 2);          Serial.print(',');
        Serial.print(theta_ekf, 2);         Serial.print(',');
        Serial.print(omega_ekf, 2);         Serial.print(',');
        Serial.print(omega_el, 2);          Serial.print(',');
        Serial.print(Ia_pred, 2);           Serial.print(',');
        Serial.print(Ib_pred, 2);           Serial.print(',');
        Serial.print(Innov_a, 2);           Serial.print(',');
        Serial.print(Innov_b, 2);           Serial.print(',');
        Serial.print(Pk_omega, 2);          Serial.print(',');
        Serial.print(Pk_theta, 2);          Serial.print(',');
        Serial.print(theta_enc, 2);         Serial.print(',');
        Serial.println(omega_enc, 2);
    }
}
