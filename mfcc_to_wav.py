"""
mfcc_to_wav.py — main.py로 저장한 MFCC 데이터셋(.npy)을 WAV로 복원

사용법:
    # 파일 경로 직접 지정
    python mfcc_to_wav.py ./wake_word/hello/hello_0000.npy

    # 파일 이름만 (--data_dir 내에서 자동 검색)
    python mfcc_to_wav.py hello_0000.npy

    # 라벨 폴더 전체 변환
    python mfcc_to_wav.py ./wake_word/hello/

    # 옵션
    python mfcc_to_wav.py hello_0000.npy --out_dir ./wavs --iter 30
"""

import sys
import argparse
import math
import struct
from pathlib import Path

import numpy as np


# ══════════════════════════════════════════════
#  설정
# ══════════════════════════════════════════════
DATASET_DIR = "./wake_word"
OUT_DIR = "."
GRIFFIN_LIM_ITER = 60

# 펌웨어 기본값
DEFAULT_SR = 16000
DEFAULT_FFT = 1024
DEFAULT_HOP = 512
DEFAULT_MELS = 15
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

    signal = 0
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


def convert_npy_to_wav(
    npy_path: Path,
    wav_path: Path,
    sr: int = DEFAULT_SR,
    fft_size: int = DEFAULT_FFT,
    hop_size: int = DEFAULT_HOP,
    n_mels: int = DEFAULT_MELS,
    n_iter: int = GRIFFIN_LIM_ITER,
) -> None:
    """단일 .npy 파일을 WAV로 변환"""
    mfcc_data = np.load(npy_path).astype(np.float64)
    n_frames, n_coeffs = mfcc_data.shape

    print(f"  입력: {npy_path.name}  shape=({n_frames}, {n_coeffs})")

    print(f"  [1/4] Inverse DCT...")
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
    print(f"  완료! ({duration_ms:.0f}ms, {len(signal)} samples)\n")


# ══════════════════════════════════════════════
#  파일 검색
# ══════════════════════════════════════════════
def find_npy_file(name_or_path: str, data_dir: str) -> Path:
    """
    경로 또는 파일 이름으로 .npy 파일을 찾습니다.

    우선순위:
        1) 그대로 경로로 존재하면 사용
        2) data_dir 아래 전체를 재귀 검색
    """
    p = Path(name_or_path)

    # 1) 직접 경로
    if p.exists() and p.is_file():
        return p

    # 2) 확장자 없으면 .npy 붙여보기
    if not p.suffix:
        p_npy = p.with_suffix(".npy")
        if p_npy.exists():
            return p_npy

    # 3) data_dir 아래 재귀 검색
    search_name = p.name if p.suffix else p.name + ".npy"
    data_root = Path(data_dir)
    matches = list(data_root.rglob(search_name))

    if len(matches) == 1:
        return matches[0]
    elif len(matches) > 1:
        print(f"[WARN] '{search_name}' 이름의 파일이 여러 개 발견됨:")
        for m in matches:
            print(f"       {m}")
        print(f"       첫 번째 파일을 사용합니다.")
        return matches[0]

    # 못 찾음
    print(f"[ERROR] 파일을 찾을 수 없습니다: {name_or_path}")
    print(f"        검색 위치: {data_root.resolve()}")
    sys.exit(1)


# ══════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        description="MFCC 데이터셋(.npy) → WAV 변환",
        epilog=(
            "예시:\n"
            "  python mfcc_to_wav.py hello_0000.npy\n"
            "  python mfcc_to_wav.py ./wake_word/hello/hello_0000.npy\n"
            "  python mfcc_to_wav.py ./wake_word/hello/   (폴더 전체 변환)\n"
            "  python mfcc_to_wav.py hello_0000            (.npy 자동 추가)\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "input",
        help=".npy 파일 경로, 파일 이름, 또는 폴더 경로 (폴더면 전체 변환)",
    )
    parser.add_argument(
        "--data_dir", default=DATASET_DIR,
        help=f"데이터셋 루트 폴더 (파일 이름 검색 시 사용, 기본: {DATASET_DIR})",
    )
    parser.add_argument(
        "--out_dir", default=None,
        help="WAV 출력 폴더 (기본: 입력 파일과 같은 폴더)",
    )
    parser.add_argument(
        "--sr", type=int, default=DEFAULT_SR, help=f"Sample rate (기본: {DEFAULT_SR})",
    )
    parser.add_argument(
        "--fft", type=int, default=DEFAULT_FFT, help=f"FFT size (기본: {DEFAULT_FFT})",
    )
    parser.add_argument(
        "--hop", type=int, default=DEFAULT_HOP, help=f"Hop size (기본: {DEFAULT_HOP})",
    )
    parser.add_argument(
        "--mels", type=int, default=DEFAULT_MELS, help=f"Mel filters (기본: {DEFAULT_MELS})",
    )
    parser.add_argument(
        "--iter", type=int, default=GRIFFIN_LIM_ITER,
        help=f"Griffin-Lim 반복 횟수 (기본: {GRIFFIN_LIM_ITER})",
    )
    args = parser.parse_args()

    input_path = Path(args.input)

    # ── 폴더가 입력된 경우: 내부 .npy 전체 변환 ──
    if input_path.is_dir():
        npy_files = sorted(input_path.glob("*.npy"))
        if not npy_files:
            print(f"[ERROR] 폴더에 .npy 파일이 없습니다: {input_path}")
            sys.exit(1)

        out_dir = Path(args.out_dir) if args.out_dir else input_path
        out_dir.mkdir(parents=True, exist_ok=True)

        print(f"[INFO] 폴더 변환: {input_path} ({len(npy_files)}개)")
        print(f"[INFO] WAV 출력 : {out_dir.resolve()}\n")

        for npy_file in npy_files:
            wav_name = npy_file.stem + ".wav"
            wav_path = out_dir / wav_name
            convert_npy_to_wav(
                npy_file, wav_path,
                sr=args.sr, fft_size=args.fft, hop_size=args.hop,
                n_mels=args.mels, n_iter=args.iter,
            )

        print(f"[INFO] 전체 변환 완료: {len(npy_files)}개")
        return

    # ── 단일 파일 ──
    npy_path = find_npy_file(args.input, args.data_dir)

    if args.out_dir:
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
    else:
        out_dir = npy_path.parent

    wav_path = out_dir / (npy_path.stem + ".wav")

    print(f"[INFO] 변환: {npy_path} → {wav_path}\n")
    convert_npy_to_wav(
        npy_path, wav_path,
        sr=args.sr, fft_size=args.fft, hop_size=args.hop,
        n_mels=args.mels, n_iter=args.iter,
    )


if __name__ == "__main__":
    main()