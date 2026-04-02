"""
mfcc_to_wav.py — COM3 시리얼에서 MFCC 수신 → WAV 실시간 변환

사용법:
    python mfcc_to_wav.py                     # 기본: COM3, 115200
    python mfcc_to_wav.py --port COM5         # 포트 변경
    python mfcc_to_wav.py --out_dir ./wavs    # 출력 폴더 변경

동작:
    1) 시리얼에서 MFCC_START ~ MFCC_END 블록을 감지
    2) Griffin-Lim으로 WAV 복원
    3) 자동 번호 매겨서 저장 (rec_000.wav, rec_001.wav, ...)
"""

import sys
import argparse
import math
import struct
from pathlib import Path

import numpy as np
import serial


# ══════════════════════════════════════════════
#  설정
# ══════════════════════════════════════════════
PORT = "COM3"
BAUD = 115200
OUT_DIR = "."
GRIFFIN_LIM_ITER = 60

# 펌웨어 기본값 (MFCC_START 헤더에서 덮어씀)
DEFAULT_SR = 16000
DEFAULT_FFT = 1024
DEFAULT_HOP = 512
DEFAULT_MELS = 15
DEFAULT_MFCC = 15
MEL_FMIN_HZ = 20.0
MEL_FMAX_HZ = 4000.0


# ══════════════════════════════════════════════
#  MFCC → WAV 변환 함수들
# ══════════════════════════════════════════════
def hz_to_mel(hz):
    return 2595.0 * np.log10(1.0 + hz / 700.0)


def mel_to_hz(mel):
    return 700.0 * (10.0 ** (mel / 2595.0) - 1.0)


def build_mel_filterbank(n_mels, fft_size, sr, fmin=MEL_FMIN_HZ, fmax=MEL_FMAX_HZ):
    """펌웨어와 동일한 mel 필터뱅크 행렬 생성"""
    n_bins = fft_size // 2 + 1
    mel_lo = hz_to_mel(fmin)
    mel_hi = hz_to_mel(fmax)
    n_edges = n_mels + 2

    mel_edges = np.zeros(n_edges, dtype=np.int32)
    for i in range(n_edges):
        t = i / (n_mels + 1)
        mel_val = mel_lo + t * (mel_hi - mel_lo)
        hz_val = mel_to_hz(mel_val)
        bin_idx = int((fft_size + 1) * hz_val / sr)
        if bin_idx > fft_size // 2:
            bin_idx = fft_size // 2
        if i > 0 and bin_idx <= mel_edges[i - 1]:
            bin_idx = mel_edges[i - 1] + 1
            if bin_idx > fft_size // 2:
                bin_idx = fft_size // 2
        mel_edges[i] = bin_idx

    fb = np.zeros((n_mels, n_bins), dtype=np.float64)
    for m in range(n_mels):
        l = mel_edges[m]
        c = mel_edges[m + 1]
        r = mel_edges[m + 2]
        if c <= l:
            c = l + 1
        if r <= c:
            r = c + 1
        for k in range(l, c):
            fb[m, k] = (k - l) / (c - l)
        for k in range(c, r):
            fb[m, k] = (r - k) / (r - c)

    return fb


def inverse_dct2(mfcc_matrix, n_mels):
    """DCT-II 역변환: MFCC → log mel 에너지"""
    n_frames, n_coeffs = mfcc_matrix.shape
    mel_log = np.zeros((n_frames, n_mels), dtype=np.float64)

    for frame_idx in range(n_frames):
        for m in range(n_mels):
            acc = 0.0
            for c in range(n_coeffs):
                acc += mfcc_matrix[frame_idx, c] * math.cos(
                    math.pi * c * (m + 0.5) / n_mels
                )
            mel_log[frame_idx, m] = acc * (2.0 / n_mels)

    return mel_log


def griffin_lim(magnitude, fft_size, hop_size, n_iter):
    """Griffin-Lim: magnitude spectrogram → 시간 신호"""
    n_frames, n_bins = magnitude.shape
    output_len = (n_frames - 1) * hop_size + fft_size

    rng = np.random.default_rng(42)
    phase = rng.uniform(-np.pi, np.pi, size=(n_frames, n_bins))

    for iteration in range(n_iter):
        complex_spec = magnitude * np.exp(1j * phase)

        signal = np.zeros(output_len, dtype=np.float64)
        window_sum = np.zeros(output_len, dtype=np.float64)
        window = np.hanning(fft_size)

        for f in range(n_frames):
            full_spec = np.zeros(fft_size, dtype=np.complex128)
            full_spec[:n_bins] = complex_spec[f]
            full_spec[n_bins:] = np.conj(complex_spec[f, -2:0:-1])

            frame_signal = np.real(np.fft.ifft(full_spec))
            start = f * hop_size
            signal[start:start + fft_size] += frame_signal * window
            window_sum[start:start + fft_size] += window ** 2

        nonzero = window_sum > 1e-8
        signal[nonzero] /= window_sum[nonzero]

        if iteration < n_iter - 1:
            for f in range(n_frames):
                start = f * hop_size
                frame = signal[start:start + fft_size] * window
                spec = np.fft.fft(frame)[:n_bins]
                phase[f] = np.angle(spec)

    return signal


def save_wav(filepath, signal, sr):
    """16-bit PCM WAV 저장"""
    peak = np.max(np.abs(signal))
    if peak > 0:
        signal = signal / peak * 0.9

    samples = np.clip(signal * 32767, -32768, 32767).astype(np.int16)
    n_samples = len(samples)
    data_size = n_samples * 2

    with open(filepath, "wb") as f:
        f.write(b"RIFF")
        f.write(struct.pack("<I", 36 + data_size))
        f.write(b"WAVE")
        f.write(b"fmt ")
        f.write(struct.pack("<I", 16))
        f.write(struct.pack("<H", 1))
        f.write(struct.pack("<H", 1))
        f.write(struct.pack("<I", sr))
        f.write(struct.pack("<I", sr * 2))
        f.write(struct.pack("<H", 2))
        f.write(struct.pack("<H", 16))
        f.write(b"data")
        f.write(struct.pack("<I", data_size))
        f.write(samples.tobytes())


def mfcc_to_wav(mfcc_data, sr, fft_size, hop_size, n_mels, wav_path, n_iter):
    """MFCC 행렬 → WAV 파일 변환 및 저장"""
    print(f"  [1/4] Inverse DCT ({mfcc_data.shape[0]} frames, {mfcc_data.shape[1]} coeffs)...")
    mel_log = inverse_dct2(mfcc_data, n_mels)

    mel_energy = np.exp(mel_log)

    print(f"  [2/4] Mel → power spectrogram...")
    mel_fb = build_mel_filterbank(n_mels, fft_size, sr)
    fb_pinv = np.linalg.pinv(mel_fb)
    power_spec = np.maximum(mel_energy @ fb_pinv.T, 0.0)
    magnitude = np.sqrt(power_spec)

    print(f"  [3/4] Griffin-Lim ({n_iter} iterations)...")
    signal = griffin_lim(magnitude, fft_size, hop_size, n_iter)

    print(f"  [4/4] WAV 저장: {wav_path}")
    save_wav(str(wav_path), signal, sr)

    duration_ms = len(signal) * 1000 / sr
    print(f"  완료! ({duration_ms:.0f}ms, {len(signal)} samples)")


# ══════════════════════════════════════════════
#  시리얼 수신 + 파싱
# ══════════════════════════════════════════════
def parse_line(line: str) -> list[str]:
    return [part.strip() for part in line.split(",")]


def main():
    parser = argparse.ArgumentParser(description="COM 시리얼 MFCC → WAV 실시간 변환")
    parser.add_argument("--port", default=PORT, help=f"시리얼 포트 (기본: {PORT})")
    parser.add_argument("--baud", type=int, default=BAUD, help=f"Baud rate (기본: {BAUD})")
    parser.add_argument("--out_dir", default=OUT_DIR, help=f"WAV 저장 폴더 (기본: {OUT_DIR})")
    parser.add_argument("--iter", type=int, default=GRIFFIN_LIM_ITER,
                        help=f"Griffin-Lim 반복 횟수 (기본: {GRIFFIN_LIM_ITER})")
    args = parser.parse_args()

    gl_iter = args.iter

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 기존 파일 번호 이어서 매기기
    existing = sorted(out_dir.glob("rec_*.wav"))
    if existing:
        last_num = int(existing[-1].stem.split("_")[1])
        rec_counter = last_num + 1
    else:
        rec_counter = 0

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

    print(f"[INFO] 시리얼 포트  : {args.port} @ {args.baud}")
    print(f"[INFO] WAV 저장 폴더: {out_dir.resolve()}")
    print(f"[INFO] 다음 파일 번호: rec_{rec_counter:03d}.wav")
    print(f"[INFO] Griffin-Lim  : {gl_iter} iterations")
    print(f"[INFO] Ctrl+C로 종료")
    print()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"[ERROR] 시리얼 포트 열기 실패: {e}")
        sys.exit(1)

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

            # ── CONFIG 파싱 ──
            if msg_type == "CONFIG":
                for pair in parts:
                    if "=" in pair:
                        k, v = pair.split("=", 1)
                        if k in cfg:
                            cfg[k] = int(v)
                print(f"[CONFIG] sr={cfg['sr']}, fft={cfg['fft']}, hop={cfg['hop']}, "
                      f"mels={cfg['mels']}, mfcc={cfg['mfcc']}")

            # ── INFO 메시지 ──
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
                print(f"[MFCC] 수집 시작 (예상 프레임: {parts[1] if len(parts) > 1 else '?'})")

            # ── MFCC 프레임 데이터 ──
            elif msg_type == "MFCC" and collecting:
                try:
                    coeffs = [float(x) for x in parts[2:]]
                    mfcc_frames.append(coeffs)
                except (ValueError, IndexError):
                    print(f"[WARN] MFCC 파싱 실패: {line}")

            # ── MFCC_END → 변환 시작 ──
            elif msg_type == "MFCC_END":
                if not collecting or not mfcc_frames:
                    print("[WARN] MFCC_END 수신했으나 데이터 없음")
                    collecting = False
                    continue

                collecting = False
                mfcc_data = np.array(mfcc_frames, dtype=np.float64)
                wav_path = out_dir / f"rec_{rec_counter:03d}.wav"

                print(f"\n{'='*50}")
                print(f"  변환 시작: rec_{rec_counter:03d}.wav")
                print(f"  프레임: {mfcc_data.shape[0]}, 계수: {mfcc_data.shape[1]}")
                print(f"  SR={cfg['sr']}, FFT={cfg['fft']}, HOP={cfg['hop']}, MEL={cfg['mels']}")
                print(f"{'='*50}")

                mfcc_to_wav(
                    mfcc_data,
                    sr=cfg["sr"],
                    fft_size=cfg["fft"],
                    hop_size=cfg["hop"],
                    n_mels=cfg["mels"],
                    wav_path=wav_path,
                    n_iter=gl_iter,
                )

                rec_counter += 1
                print(f"\n[INFO] 다음 녹음 대기중... (다음: rec_{rec_counter:03d}.wav)\n")

            # ── 기타 ──
            else:
                if line:
                    print(f"[>] {line}")

    except KeyboardInterrupt:
        print(f"\n[INFO] 종료됨. 총 {rec_counter}개 WAV 저장됨.")
    finally:
        ser.close()
        print("[INFO] 시리얼 포트 닫힘.")


if __name__ == "__main__":
    main()