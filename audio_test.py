"""
音频设备测试脚本（基于 ALSA arecord/aplay）
- 检测麦克风和扬声器
- 录音 3 秒
- 播放录音
- 播放合成正弦波
"""

import subprocess
import struct
import math
import wave
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WAV_RECORD = os.path.join(SCRIPT_DIR, "test_recording.wav")
WAV_TONE = os.path.join(SCRIPT_DIR, "test_tone.wav")


def run(cmd, timeout=15):
    """执行命令并返回 (returncode, stdout, stderr)"""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout, r.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "超时"
    except Exception as e:
        return -1, "", str(e)


def list_devices():
    print("=" * 60)
    print("音频设备检测")
    print("=" * 60)

    print("\n【录音设备（麦克风）】")
    rc, out, err = run(["arecord", "-l"])
    print(out if rc == 0 else f"  检测失败: {err}")

    print("【播放设备（扬声器）】")
    rc, out, err = run(["aplay", "-l"])
    print(out if rc == 0 else f"  检测失败: {err}")

    print("【PulseAudio 输入源】")
    rc, out, _ = run(["pactl", "list", "sources", "short"])
    print(out if rc == 0 else "  pactl 不可用")

    print("【PulseAudio 输出】")
    rc, out, _ = run(["pactl", "list", "sinks", "short"])
    print(out if rc == 0 else "  pactl 不可用")


def generate_sine_wav(filename, freq=440, duration=2, samplerate=44100, amplitude=0.3):
    """生成正弦波 WAV 文件"""
    n_samples = int(samplerate * duration)
    with wave.open(filename, "w") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(samplerate)
        for i in range(n_samples):
            sample = int(amplitude * 32767 * math.sin(2 * math.pi * freq * i / samplerate))
            wf.writeframes(struct.pack("<h", sample))
    return filename


def test_play_sine(freq=440, duration=2):
    print("=" * 60)
    print(f"扬声器测试：播放 {freq}Hz 正弦波 {duration} 秒")
    print("=" * 60)

    generate_sine_wav(WAV_TONE, freq=freq, duration=duration)
    print(f"  已生成测试音频: {WAV_TONE}")

    rc, out, err = run(["aplay", WAV_TONE], timeout=duration + 5)
    if rc == 0:
        print("  ✓ 播放完成（如果听到 '嘟' 声说明扬声器正常）")
        return True
    else:
        print(f"  ✗ 播放失败: {err.strip()}")
        return False


def test_record(duration=3, samplerate=44100):
    print("=" * 60)
    print(f"麦克风测试：录音 {duration} 秒")
    print("=" * 60)

    cmd = [
        "arecord",
        "-d", str(duration),
        "-f", "S16_LE",
        "-r", str(samplerate),
        "-c", "1",
        WAV_RECORD,
    ]
    rc, out, err = run(cmd, timeout=duration + 5)
    if rc != 0:
        print(f"  ✗ 录音失败: {err.strip()}")
        return False

    print(f"  录音完成，已保存到: {WAV_RECORD}")

    try:
        with wave.open(WAV_RECORD, "r") as wf:
            frames = wf.readframes(wf.getnframes())
            samples = struct.unpack(f"<{len(frames)//2}h", frames)
            peak = max(abs(s) for s in samples)
            rms = math.sqrt(sum(s * s for s in samples) / len(samples))
            print(f"  采样点: {len(samples)}, 峰值: {peak}, RMS: {rms:.1f}")

            if peak < 50:
                print("  ⚠ 信号极弱，麦克风可能未连接或静音")
            else:
                print("  ✓ 检测到音频信号，麦克风正常")
    except Exception as e:
        print(f"  分析录音失败: {e}")

    return True


def test_playback():
    print("=" * 60)
    print(f"回放测试：播放刚才的录音")
    print("=" * 60)

    if not os.path.exists(WAV_RECORD):
        print(f"  ✗ 录音文件不存在: {WAV_RECORD}")
        return False

    rc, out, err = run(["aplay", WAV_RECORD], timeout=10)
    if rc == 0:
        print("  ✓ 回放完成（如果听到声音说明扬声器和录音均正常）")
        return True
    else:
        print(f"  ✗ 回放失败: {err.strip()}")
        return False


if __name__ == "__main__":
    list_devices()

    results = {}

    print("\n>>> 测试 1/3: 扬声器 - 正弦波")
    results["扬声器"] = test_play_sine()

    print("\n>>> 测试 2/3: 麦克风 - 录音 3 秒")
    results["麦克风"] = test_record()

    print("\n>>> 测试 3/3: 扬声器 - 回放录音")
    results["回放"] = test_playback()

    print("\n" + "=" * 60)
    print("测试结果汇总")
    print("=" * 60)
    for name, ok in results.items():
        print(f"  {name}: {'✓ 通过' if ok else '✗ 失败'}")
    print()
