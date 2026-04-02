"""
main.py — COM3 시리얼에서 MFCC 수신 → wake-word 학습 데이터셋 저장

사용법:
    python main.py                            # 기본: hello 라벨
    python main.py --label noise              # noise 라벨로 수집
    python main.py --port COM5                # 포트 변경

데이터셋 구조:
    ./wake_word/
    ├── hello/
    │   ├── hello_000.npy       # shape: (45, 15)  원본 MFCC
    │   ├── hello_001.npy
    │   └── ...
    ├── noise/                  # --label noise 로 수집
    │   ├── noise_000.npy
    │   └── ...
    └── metadata.csv            # 전체 샘플 목록

대상 모델 구조:
    1D temporal CNN → INT8 wake-word binary classifier
    ┌─────────────────────────────────────────────┐
    │  Input         : (32, 15)                   │
    │  Conv1D        : 24 filters, kernel=5       │
    │  DS-Conv block : ×2  (depthwise-separable)  │
    │  GAP           : global average pooling      │
    │  Dense         : 2  → background / wake_word │
    │  Quantization  : INT8                        │
    └─────────────────────────────────────────────┘

    - 원본 MFCC는 (45, 15)로 저장하고, 학습 시 center-crop/pad → (32, 15)
    - .npy float32 형식으로 저장하여 학습 파이프라인에서 바로 로드 가능
"""

import csv
import sys
import argparse
from pathlib import Path

import numpy as np
import serial


# ══════════════════════════════════════════════
#  설정
# ══════════════════════════════════════════════
PORT = "COM3"
BAUD = 115200
DATASET_DIR = "./wake_word"

# ── 라벨 설정 ──
# 현재 수집 대상:
#   "hello"      : wake word 발화 샘플
#
# 향후 추가 수집 가능한 라벨:
#   "noise"      : 배경 소음 (에어컨, 키보드, TV 등)
#   "silence"    : 무음 / 매우 조용한 환경
#   "unknown"    : wake word가 아닌 일반 발화 ("네", "아니", "오케이" 등)
#
# 사용 예시:
#   python main.py --label hello     # wake word 수집 (기본)
#   python main.py --label noise     # 배경 소음 수집
#   python main.py --label unknown   # 비-wake-word 발화 수집
DEFAULT_LABEL = "hello"

# ── 모델 입력 사양 (참고용, 저장 시에는 원본 그대로 저장) ──
MODEL_INPUT_FRAMES = 32
MODEL_INPUT_COEFFS = 15

# ── 펌웨어 기본값 ──
DEFAULT_SR = 16000
DEFAULT_FFT = 1024
DEFAULT_HOP = 512
DEFAULT_MELS = 15
DEFAULT_MFCC = 15


# ══════════════════════════════════════════════
#  데이터셋 유틸리티
# ══════════════════════════════════════════════
METADATA_FILE = "metadata.csv"
METADATA_HEADER = ["id", "label", "filename", "n_frames", "n_coeffs", "sr", "fft", "hop"]


def ensure_dirs(dataset_dir: Path, label: str) -> Path:
    """라벨 디렉토리 생성 및 반환"""
    label_dir = dataset_dir / label
    label_dir.mkdir(parents=True, exist_ok=True)
    return label_dir


def ensure_metadata(dataset_dir: Path) -> None:
    """metadata.csv 헤더가 없으면 생성"""
    meta_path = dataset_dir / METADATA_FILE
    if meta_path.exists() and meta_path.stat().st_size > 0:
        return
    dataset_dir.mkdir(parents=True, exist_ok=True)
    with meta_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(METADATA_HEADER)


def get_next_id(dataset_dir: Path) -> int:
    """metadata.csv에서 다음 ID 번호를 반환"""
    meta_path = dataset_dir / METADATA_FILE
    if not meta_path.exists() or meta_path.stat().st_size == 0:
        return 0

    last_id = -1
    with meta_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        next(reader, None)  # 헤더 스킵
        for row in reader:
            if not row:
                continue
            try:
                last_id = max(last_id, int(row[0]))
            except (ValueError, IndexError):
                continue

    return last_id + 1


def get_next_file_number(label_dir: Path, label: str) -> int:
    """라벨 폴더 내 기존 파일에서 다음 번호를 반환"""
    existing = sorted(label_dir.glob(f"{label}_*.npy"))
    if not existing:
        return 0
    try:
        last_num = int(existing[-1].stem.split("_")[-1])
        return last_num + 1
    except (ValueError, IndexError):
        return len(existing)


def save_sample(
    mfcc_data: np.ndarray,
    label: str,
    sample_id: int,
    file_num: int,
    label_dir: Path,
    dataset_dir: Path,
    cfg: dict,
) -> str:
    """
    MFCC 데이터를 .npy로 저장하고 metadata.csv에 기록

    저장 형식:
        - shape: (n_frames, n_coeffs)  예: (45, 15)
        - dtype: float32
        - 모델 학습 시 (32, 15)로 center-crop 또는 pad 처리
    """
    filename = f"{label}_{file_num:04d}.npy"
    filepath = label_dir / filename

    # float32로 저장 (INT8 양자화 학습 파이프라인과 호환)
    np.save(filepath, mfcc_data.astype(np.float32))

    # metadata.csv에 추가
    meta_path = dataset_dir / METADATA_FILE
    with meta_path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            sample_id,
            label,
            f"{label}/{filename}",
            mfcc_data.shape[0],   # n_frames (원본, 예: 45)
            mfcc_data.shape[1],   # n_coeffs (예: 15)
            cfg["sr"],
            cfg["fft"],
            cfg["hop"],
        ])

    return filename


# ══════════════════════════════════════════════
#  시리얼 파싱
# ══════════════════════════════════════════════
def parse_line(line: str) -> list[str]:
    return [part.strip() for part in line.split(",")]


# ══════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        description="COM 시리얼 MFCC → wake-word 학습 데이터셋 저장"
    )
    parser.add_argument("--port", default=PORT, help=f"시리얼 포트 (기본: {PORT})")
    parser.add_argument("--baud", type=int, default=BAUD, help=f"Baud rate (기본: {BAUD})")
    parser.add_argument("--out_dir", default=DATASET_DIR, help=f"데이터셋 폴더 (기본: {DATASET_DIR})")
    parser.add_argument(
        "--label", default=DEFAULT_LABEL,
        help=(
            f"수집할 라벨 (기본: {DEFAULT_LABEL}). "
            "예: hello, noise, silence, unknown"
        ),
    )
    args = parser.parse_args()

    dataset_dir = Path(args.out_dir)
    label = args.label
    label_dir = ensure_dirs(dataset_dir, label)
    ensure_metadata(dataset_dir)

    next_id = get_next_id(dataset_dir)
    file_num = get_next_file_number(label_dir, label)

    # MFCC 수집 상태
    collecting = False
    mfcc_frames = []
    cfg = {
        "sr": DEFAULT_SR,
        "fft": DEFAULT_FFT,
        "hop": DEFAULT_HOP,
        "mels": DEFAULT_MELS,
        "mfcc": DEFAULT_MFCC,
    }

    print(f"[INFO] 시리얼 포트    : {args.port} @ {args.baud}")
    print(f"[INFO] 데이터셋 폴더  : {dataset_dir.resolve()}")
    print(f"[INFO] 라벨           : {label}")
    print(f"[INFO] 다음 ID        : {next_id}")
    print(f"[INFO] 다음 파일      : {label}_{file_num:04d}.npy")
    print(f"[INFO] 모델 입력 사양 : ({MODEL_INPUT_FRAMES}, {MODEL_INPUT_COEFFS})")
    print(f"[INFO] Ctrl+C로 종료")
    print()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"[ERROR] 시리얼 포트 열기 실패: {e}")
        sys.exit(1)

    saved_count = 0

    try:
        while True:
            try:
                raw = ser.readline()
            except serial.SerialException as e:
                print(f"[ERROR] 시리얼 읽기 실패: {e}")
                break

            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            parts = parse_line(line)
            if not parts:
                continue

            msg_type = parts[0]

            # ── CONFIG ──
            if msg_type == "CONFIG":
                for pair in parts:
                    if "=" in pair:
                        k, v = pair.split("=", 1)
                        if k in cfg:
                            cfg[k] = int(v)
                print(
                    f"[CONFIG] sr={cfg['sr']}, fft={cfg['fft']}, hop={cfg['hop']}, "
                    f"mels={cfg['mels']}, mfcc={cfg['mfcc']}"
                )

            # ── INFO ──
            elif msg_type == "INFO":
                info_msg = ",".join(parts[1:]) if len(parts) > 1 else ""
                print(f"[INFO] {info_msg}")

            # ── ERROR ──
            elif msg_type == "ERROR":
                err_msg = ",".join(parts[1:]) if len(parts) > 1 else ""
                print(f"[ERROR] {err_msg}")

            # ── REC_DONE ──
            elif msg_type == "REC_DONE":
                detail = ",".join(parts[1:]) if len(parts) > 1 else ""
                print(f"[REC_DONE] {detail}")

            # ── MFCC_START ──
            elif msg_type == "MFCC_START":
                collecting = True
                mfcc_frames = []
                if len(parts) >= 6:
                    cfg["sr"] = int(parts[3])
                    cfg["fft"] = int(parts[4])
                    cfg["hop"] = int(parts[5])
                    cfg["mfcc"] = int(parts[2])
                expected = parts[1] if len(parts) > 1 else "?"
                print(f"[MFCC] 수집 시작 (예상 프레임: {expected})")

            # ── MFCC 프레임 데이터 ──
            elif msg_type == "MFCC" and collecting:
                try:
                    coeffs = [float(x) for x in parts[2:]]
                    mfcc_frames.append(coeffs)
                except (ValueError, IndexError):
                    print(f"[WARN] MFCC 파싱 실패: {line}")

            # ── MFCC_END → 데이터셋 저장 ──
            elif msg_type == "MFCC_END":
                if not collecting or not mfcc_frames:
                    print("[WARN] MFCC_END 수신했으나 데이터 없음")
                    collecting = False
                    continue

                collecting = False
                mfcc_data = np.array(mfcc_frames, dtype=np.float64)
                n_frames, n_coeffs = mfcc_data.shape

                # 저장
                filename = save_sample(
                    mfcc_data, label, next_id, file_num,
                    label_dir, dataset_dir, cfg,
                )

                saved_count += 1
                print(f"\n{'='*50}")
                print(f"  저장 완료: {label}/{filename}")
                print(f"  ID: {next_id}, shape: ({n_frames}, {n_coeffs})")
                print(f"  모델 입력 시: center-crop/pad → ({MODEL_INPUT_FRAMES}, {MODEL_INPUT_COEFFS})")
                print(f"  누적 저장: {saved_count}개 ({label})")
                print(f"{'='*50}\n")

                next_id += 1
                file_num += 1
                print(f"[INFO] 다음 녹음 대기중... (다음: {label}_{file_num:04d}.npy)\n")

            # ── 기타 ──
            else:
                if line:
                    print(f"[>] {line}")

    except KeyboardInterrupt:
        print(f"\n[INFO] 종료됨. 이번 세션 저장: {saved_count}개 ({label})")
    finally:
        ser.close()
        print("[INFO] 시리얼 포트 닫힘.")


if __name__ == "__main__":
    main()