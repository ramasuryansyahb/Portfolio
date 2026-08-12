import serial
import csv
import time
import os
import threading

# ─── KONFIGURASI ──────────────────────────────────────────────────────────────
COM_PORT  = 'COM3'        
BAUD_RATE = 115200        
SAVE_DIR  = r'D:\MPU9250_Data'
# ──────────────────────────────────────────────────────────────────────────────


def close_file(file_obj):
    """Helper: tutup file kalau masih terbuka."""
    if file_obj is not None:
        try:
            file_obj.close()
        except Exception:
            pass


def make_filename():
    return os.path.join(SAVE_DIR, f"data_MPU9250_{int(time.time())}.csv")


def input_listener(ser):
    """
    Thread terpisah: baca input keyboard dan kirim ke ESP32.
    Ketik 'r' + Enter untuk mulai sesi baru, 's' + Enter untuk stop.
    """
    while True:
        try:
            cmd = input()
            if cmd.strip().lower() in ('r', 's'):
                ser.write(cmd.strip().lower().encode('utf-8'))
        except (EOFError, Exception):
            break


def main():
    os.makedirs(SAVE_DIR, exist_ok=True)

    print("\n[PYTHON STANDBY AKTIF]")
    print(f"LOKASI SAVE CSV : {SAVE_DIR}")
    print(f"PORT            : {COM_PORT}  |  BAUD: {BAUD_RATE}")
    print("Menunggu koneksi ke ESP32...")
    print("Ketik 'r' + Enter kapan saja untuk mulai sesi baru, 's' + Enter untuk stop.\n")

    file          = None
    writer        = None
    expected_cols = 0        # diisi dinamis dari header Arduino
    wait_header   = False    # flag: baris berikutnya adalah header CSV

    while True:
        try:
            ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1.0)
            print(f">>> TERHUBUNG ke {COM_PORT} — menunggu START_LOG ...\n")

            listener = threading.Thread(target=input_listener, args=(ser,), daemon=True)
            listener.start()

            while True:
                if not ser.in_waiting:
                    continue

                raw  = ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()

                if not line:
                    continue

                # ── START_LOG : buka file baru ────────────────────────────
                if "START_LOG" in line:
                    close_file(file)
                    file          = None
                    writer        = None
                    expected_cols = 0
                    wait_header   = True

                    fname = make_filename()
                    file  = open(fname, mode='w', newline='', encoding='utf-8')
                    writer = csv.writer(file)

                    print(f"\n>>> [START_LOG] File baru: {fname}")
                    print("-" * 72)
                    continue

                # ── STOP_LOG : tutup file ─────────────────────────────────
                if "STOP_LOG" in line:
                    close_file(file)
                    file          = None
                    writer        = None
                    expected_cols = 0
                    wait_header   = False

                    print("\n>>> [STOP_LOG] Logging selesai — file ditutup.")
                    print("    Ketik 'r' + Enter untuk mulai sesi baru...\n")
                    continue

                # ── Baris pertama setelah START_LOG = header CSV ──────────
                if wait_header:
                    if writer is None:
                        continue

                    cols = [c.strip() for c in line.split(',')]
                    expected_cols = len(cols)
                    writer.writerow(cols)
                    file.flush()
                    wait_header = False

                    print(f"    Header terdeteksi  : {expected_cols} kolom")
                    print(f"    Kolom               : {cols}")
                    print("-" * 72)
                    continue

                # ── Baris data biasa ──────────────────────────────────────
                if writer is not None and expected_cols > 0:
                    parts = line.split(',')

                    if len(parts) != expected_cols:
                        print(f"    [SKIP] {len(parts)} kolom (ekspektasi {expected_cols}): {line[:60]}")
                        continue

                    writer.writerow(parts)
                    file.flush()

                   try:
                        print(
                            f"  T={parts[0]:>8}s | "
                            f"Roll={parts[10]:>7} | "
                            f"Pitch={parts[11]:>7} | "
                            f"Yaw={parts[12]:>7} | "
                            f"AccZ={parts[3]:>7} | "
                            f"GyroZ={parts[6]:>7}"
                        )
                    except IndexError:
                        print(f"  Rec: {line[:80]}")

        except serial.SerialException:
            close_file(file)
            file          = None
            writer        = None
            expected_cols = 0
            wait_header   = False
            print(f"\n[{COM_PORT} TERPUTUS] Auto-reconnect dalam 1.5 s ...")
            time.sleep(1.5)

        except KeyboardInterrupt:
            print("\n>>> Dihentikan manual. Menutup program...")
            break

        except Exception as exc:
            print(f"[WARNING] Error tidak terduga: {exc}")
            time.sleep(1.0)

    close_file(file)
    print("Program selesai.")


if __name__ == "__main__":
    main()
