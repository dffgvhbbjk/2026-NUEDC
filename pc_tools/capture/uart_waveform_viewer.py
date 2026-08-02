# -*- coding: utf-8 -*-
# ============================================================================
# uart_waveform_viewer.py -- 串口波形实时查看器
#
# 用法:
#   实时串口:  python uart_waveform_viewer.py --port COM5
#   离线文件:  python uart_waveform_viewer.py --file capture.bin
#
# 帧格式: AA 55 len_sel [samples*3N] chksum [result_segment 13 bytes]
#   每个样点 24-bit 有符号大端, N = 64/128/256/512
#
# 依赖: pyserial, matplotlib
# ============================================================================

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import argparse
import os
import sys
import time
from collections import deque

# ---------------------------------------------------------------------------
# 帧解析 (与 uart_frame_parser.py 保持一致)
# ---------------------------------------------------------------------------
RESULT_VER = 0x02
LEN_TABLE = {0: 64, 1: 128, 2: 256, 3: 512}
STATE_NAME = {0: "INVALID", 1: "NORMAL", 2: "DEFECT", 3: "RSV3"}
TAIL_LEN = 13


def s24(b2, b1, b0):
    """大端 3 字节 -> 有符号 24 位整数"""
    v = (b2 << 16) | (b1 << 8) | b0
    return v - (1 << 24) if v & 0x800000 else v


def try_parse(buf):
    """解析一帧, 返回 (frame_dict, consumed) 或 (None, skip)"""
    idx = buf.find(b"\xAA\x55")
    if idx < 0:
        return None, max(0, len(buf) - 1)
    if idx > 0:
        return None, idx

    if len(buf) < 3:
        return None, 0
    len_sel = buf[2]
    if len_sel not in LEN_TABLE:
        return None, 1
    n = LEN_TABLE[len_sel]
    total = 3 + 3 * n + 1 + TAIL_LEN
    if len(buf) < total:
        return None, 0

    data = buf[3:3 + 3 * n]
    chksum = buf[3 + 3 * n]
    calc = (len_sel + sum(data)) & 0xFF
    if calc != chksum:
        return None, 1

    tail = buf[3 + 3 * n + 1: total]
    if tail[0] != 0x5A or tail[1] != RESULT_VER:
        return None, 1
    rsum = sum(tail[1:12]) & 0xFF
    if rsum != tail[12]:
        return None, 1

    samples = [s24(data[i], data[i + 1], data[i + 2])
               for i in range(0, 3 * n, 3)]
    status = tail[2]
    idx_h = tail[3]
    frame = {
        "n": n,
        "samples": samples,
        "result_ready": (status >> 4) & 1,
        "state": status & 0x3,
        "confidence": (status >> 2) & 0x3,
        "impact_index": ((idx_h & 1) << 8) | tail[4],
        "defect_index": (((idx_h >> 1) & 1) << 8) | tail[5],
        "bottom_index": (((idx_h >> 2) & 1) << 8) | tail[6],
        "distance_mm": ((tail[7] & 0xF) << 8) | tail[8],
        "threshold": (tail[9] << 16) | (tail[10] << 8) | tail[11],
    }
    return frame, total


# ---------------------------------------------------------------------------
# 绘图
# ---------------------------------------------------------------------------

def setup_plot(max_points=512):
    """初始化 matplotlib 图形"""
    import matplotlib.pyplot as plt
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.set_xlabel("sample index")
    ax.set_ylabel("ADC value (24-bit signed)")
    ax.set_title("UART Waveform Viewer — waiting for frame...")
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='gray', linewidth=0.5)
    line, = ax.plot([], [], 'b-', linewidth=0.8)
    # 标记线
    impact_line = ax.axvline(x=0, color='r', linestyle='--', linewidth=0.8, alpha=0.7)
    defect_line = ax.axvline(x=0, color='orange', linestyle='--', linewidth=0.8, alpha=0.7)
    bottom_line = ax.axvline(x=0, color='green', linestyle='--', linewidth=0.8, alpha=0.7)
    ax.set_xlim(0, max_points)
    return fig, ax, line, impact_line, defect_line, bottom_line


def update_plot(ax, line, impact_line, defect_line, bottom_line,
                frame, frame_count):
    """更新绘图"""
    import matplotlib.pyplot as plt
    samples = frame["samples"]
    n = len(samples)
    x = list(range(n))

    line.set_data(x, samples)
    ax.set_xlim(0, n)

    # 自动缩放 Y 轴
    peak = max(abs(min(samples)), abs(max(samples))) or 1
    margin = peak * 0.1
    ax.set_ylim(-peak - margin, peak + margin)

    # 更新标记线
    trigger_idx = n // 8  # pre-trigger 位置
    impact_line.set_xdata([frame["impact_index"]])
    defect_line.set_xdata([frame["defect_index"]])
    bottom_line.set_xdata([frame["bottom_index"]])

    # 标题
    state_str = STATE_NAME.get(frame["state"], "?")
    title = (f"[{frame_count}] {n}-point  |  "
             f"state={state_str}  |  "
             f"impact={frame['impact_index']}  "
             f"defect={frame['defect_index']}  "
             f"bottom={frame['bottom_index']}  |  "
             f"dist={frame['distance_mm']}mm  |  "
             f"thr={frame['threshold']}")
    ax.set_title(title)

    plt.pause(0.01)


# ---------------------------------------------------------------------------
# 主循环
# ---------------------------------------------------------------------------

def ensure_module(name, pip_name=None):
    """确保第三方库已安装，否则自动安装"""
    if pip_name is None:
        pip_name = name
    try:
        __import__(name)
    except ImportError:
        import subprocess
        print(f"正在安装 {pip_name} ...")
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", pip_name],
            stdout=sys.stdout, stderr=sys.stderr)
        print(f"安装完成: {pip_name}\n")


def run_serial(port, baud, max_frames=None):
    ensure_module("serial", "pyserial")
    ensure_module("matplotlib")

    import serial
    import matplotlib.pyplot as plt

    fig, ax, line, impact_line, defect_line, bottom_line = setup_plot()
    plt.show(block=False)        # 非阻塞显示窗口
    plt.pause(0.2)               # 给窗口渲染时间

    try:
        ser = serial.Serial(port, baud, timeout=0.01)  # 短超时, 不让 GUI 卡住
    except serial.SerialException as e:
        print(f"⚠ 无法打开串口 {port}: {e}")
        try:
            from serial.tools import list_ports
            ports = list(list_ports.comports())
            if ports:
                print("  可用串口:")
                for p in ports:
                    print(f"    {p.device}  --  {p.description}")
            else:
                print("  未检测到任何串口!")
        except Exception:
            pass
        plt.close(fig)
        input("\n按回车退出...")
        return

    print(f"监听 {port} @ {baud} bps, 关闭绘图窗口或 Ctrl+C 退出\n")
    buf = bytearray()
    frame_count = 0
    dropped = 0
    last_rx = time.monotonic()

    try:
        while plt.fignum_exists(fig.number):
            chunk = ser.read(4096)
            now = time.monotonic()

            if chunk:
                buf += chunk
                last_rx = now
            elif buf and (now - last_rx) > 0.5:
                dropped += len(buf)
                buf.clear()

            # 解析缓冲区内所有可用帧
            while buf:
                frame, consumed = try_parse(bytes(buf))
                if frame is not None:
                    frame_count += 1
                    update_plot(ax, line, impact_line, defect_line,
                                bottom_line, frame, frame_count)
                    del buf[:consumed]
                    print(f"  [{frame_count}] {frame['n']}pt  "
                          f"state={STATE_NAME[frame['state']]}  "
                          f"dist={frame['distance_mm']}mm")
                    if max_frames and frame_count >= max_frames:
                        break
                elif consumed > 0:
                    dropped += consumed
                    del buf[:consumed]
                else:
                    break

            if max_frames and frame_count >= max_frames:
                print(f"\n已采集 {frame_count} 帧, 完成.")
                break

            # 关键: 持续驱动 GUI 事件循环, 防止窗口假死
            plt.pause(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    print(f"\n共 {frame_count} 帧, 丢弃 {dropped} 字节")
    plt.close(fig)
    input("\n按回车退出...")


def run_file(fname):
    import matplotlib.pyplot as plt

    with open(fname, "rb") as f:
        buf = f.read()

    fig, ax, line, impact_line, defect_line, bottom_line = setup_plot()
    fig.show()

    frame_count = 0
    pos = 0
    dropped = 0

    while pos < len(buf):
        frame, consumed = try_parse(buf[pos:])
        if frame is not None:
            frame_count += 1
            update_plot(ax, line, impact_line, defect_line,
                        bottom_line, frame, frame_count)
            pos += consumed
            print(f"  [{frame_count}] {frame['n']}pt  "
                  f"state={STATE_NAME[frame['state']]}  "
                  f"dist={frame['distance_mm']}mm")
            input(f"  按回车看下一帧 (第 {frame_count} 帧) ...")
        elif consumed > 0:
            dropped += consumed
            pos += consumed
        else:
            break

    print(f"\n共 {frame_count} 帧, 丢弃 {dropped} 字节, "
          f"尾部残留 {len(buf) - pos} 字节")

    if frame_count > 0:
        input("\n按回车退出...")
    else:
        print("未解析到有效帧!")


def main():
    ap = argparse.ArgumentParser(description="UART 波形实时查看器")
    src = ap.add_mutually_exclusive_group(required=False)
    src.add_argument("--port", help="串口号, 如 COM5")
    src.add_argument("--file", help="离线二进制文件")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--max-frames", type=int, default=None,
                    help="采集 N 帧后自动停止 (仅串口模式)")
    args = ap.parse_args()

    # 没给参数 → 交互模式 (双击运行)
    if not args.port and not args.file:
        print("=" * 50)
        print("  UART 波形实时查看器")
        print("=" * 50)
        print()
        # 列出可用串口
        try:
            from serial.tools import list_ports
            ports = list(list_ports.comports())
            if ports:
                print("检测到以下串口:")
                for p in ports:
                    print(f"  {p.device}  --  {p.description}")
            else:
                print("未检测到串口! 请确认 FPGA 板已连接。")
        except Exception:
            pass
        print()
        choice = input("串口号 (如 COM5) 或 .bin 文件路径: ").strip()
        if not choice:
            print("未输入, 退出。")
            input("\n按回车退出...")
            return
        if choice.lower().endswith(".bin"):
            args.file = choice
        else:
            if choice.isdigit():
                choice = "COM" + choice
            args.port = choice

    try:
        if args.file:
            run_file(args.file)
        else:
            run_serial(args.port, args.baud, args.max_frames)
    except Exception as e:
        print(f"\n⚠ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        input("\n按回车退出...")


if __name__ == "__main__":
    main()
