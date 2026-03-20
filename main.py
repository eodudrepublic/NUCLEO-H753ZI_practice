import csv
import sys
from pathlib import Path

import serial


PORT = "COM3"
BAUD = 115200
CSV_FILE = "imu_data.csv"
EXPECTED_SAMPLE_COUNT = 20


def get_next_id(csv_path: Path) -> int:
    if not csv_path.exists():
        return 0

    last_id = None

    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader, None)

        for row in reader:
            if not row:
                continue
            try:
                last_id = int(row[0])
            except (ValueError, IndexError):
                continue

    if last_id is None:
        return 0

    return last_id + 1


def ensure_csv_header(csv_path: Path) -> None:
    if csv_path.exists() and csv_path.stat().st_size > 0:
        return

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["ID", "sample_idx", "ax", "ay", "az", "gx", "gy", "gz"])


def parse_line(line: str) -> list[str]:
    return [part.strip() for part in line.split(",")]


def is_info_start(parts: list[str]) -> bool:
    if not parts or parts[0] != "INFO":
        return False

    if len(parts) >= 2 and parts[1] == "START":
        return True

    text = " ".join(parts[1:]).upper()
    return "START" in text


def is_info_end(parts: list[str]) -> bool:
    if not parts or parts[0] != "INFO":
        return False

    if len(parts) >= 2 and parts[1] == "END":
        return True

    text = " ".join(parts[1:]).upper()
    return "END" in text


def main():
    csv_path = Path(CSV_FILE)
    ensure_csv_header(csv_path)

    next_id = get_next_id(csv_path)
    current_id = None
    collecting = False
    collected_count = 0

    print(f"[INFO] Open serial port: {PORT} @ {BAUD}")
    print(f"[INFO] Output CSV file : {csv_path.resolve()}")
    print(f"[INFO] Next ID         : {next_id}")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"[ERROR] Failed to open serial port: {e}")
        sys.exit(1)

    try:
        with csv_path.open("a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)

            while True:
                try:
                    raw = ser.readline()
                except serial.SerialException as e:
                    print(f"[ERROR] Serial read failed: {e}")
                    break

                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                print(line)
                parts = parse_line(line)

                if not parts:
                    continue

                msg_type = parts[0]

                # 1) ERROR -> 프로그램 종료
                if msg_type == "ERROR":
                    print("[ERROR] ERROR message received from MCU. Program will terminate.")
                    break

                # 2) INFO -> START / END 처리
                if msg_type == "INFO":
                    if is_info_start(parts):
                        if collecting:
                            print("[WARN] START received while already collecting. Previous session will be discarded logically.")
                        current_id = next_id
                        next_id += 1
                        collecting = True
                        collected_count = 0
                        print(f"[INFO] Data collection started. ID = {current_id}")

                    elif is_info_end(parts):
                        if collecting:
                            print(f"[INFO] Data collection ended. ID = {current_id}, samples = {collected_count}")
                            if collected_count != EXPECTED_SAMPLE_COUNT:
                                print(f"[WARN] Expected {EXPECTED_SAMPLE_COUNT} samples, but got {collected_count}.")
                        else:
                            print("[WARN] END received while not collecting.")

                        collecting = False
                        current_id = None
                        collected_count = 0

                    else:
                        # 기타 INFO는 무시
                        pass

                    continue

                # 3) DATA -> 데이터 저장
                if msg_type == "DATA":
                    if not collecting or current_id is None:
                        print("[WARN] DATA received outside collection state. Ignored.")
                        continue

                    # 기대 형식:
                    # DATA, sample_idx, ax, ay, az, gx, gy, gz
                    if len(parts) != 8:
                        print(f"[WARN] Invalid DATA format: {parts}")
                        continue

                    try:
                        sample_idx = int(parts[1])
                        ax = int(parts[2])
                        ay = int(parts[3])
                        az = int(parts[4])
                        gx = int(parts[5])
                        gy = int(parts[6])
                        gz = int(parts[7])
                    except ValueError:
                        print(f"[WARN] DATA parse failed: {parts}")
                        continue

                    writer.writerow([current_id, sample_idx, ax, ay, az, gx, gy, gz])
                    f.flush()
                    collected_count += 1
                    continue

                # 기타 메시지는 무시
                print(f"[WARN] Unknown line type: {parts}")

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        ser.close()
        print("[INFO] Serial port closed.")


if __name__ == "__main__":
    main()