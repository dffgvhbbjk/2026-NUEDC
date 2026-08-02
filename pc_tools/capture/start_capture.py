# -*- coding: utf-8 -*-
# ============================================================================
# start_capture.py -- 采集启动器 (Python 版)
#
# 双击运行或命令行: python start_capture.py
# 自动检查 pyserial, 询问串口号, 然后启动 uart_frame_parser.py
# ============================================================================

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import os
import subprocess
import sys


def list_com_ports():
    """列出当前可用的串口"""
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        if ports:
            print("检测到以下串口:")
            for p in ports:
                print(f"  {p.device}  --  {p.description}")
        else:
            print("未检测到任何串口!")
        return ports
    except Exception:
        return []


def main():
    try:
        # 1. 检查 pyserial
        try:
            import serial  # noqa: F401
        except ImportError:
            print("首次运行, 正在安装 pyserial ...")
            subprocess.check_call(
                [sys.executable, "-m", "pip", "install", "pyserial"],
                stdout=sys.stdout, stderr=sys.stderr,
            )
            print("安装完成.\n")

        # 2. 列出可用串口
        ports = list_com_ports()
        print()

        # 3. 询问串口号
        try:
            comport = input("串口号 (直接回车默认 COM5): ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n已取消.")
            input("\n按回车退出...")
            return
        if not comport:
            comport = "COM5"
        elif comport.isdigit():
            comport = "COM" + comport

        # 4. 启动 uart_frame_parser.py
        script_dir = os.path.dirname(os.path.abspath(__file__))
        parser = os.path.join(script_dir, "uart_frame_parser.py")
        outdir = os.path.join(script_dir, "..", "data")

        cmd = [sys.executable, parser, "--port", comport, "--outdir", outdir]
        print(f"\n启动: {' '.join(cmd)}\n")
        try:
            result = subprocess.run(cmd)
            if result.returncode != 0:
                print(f"\n⚠ 程序异常退出 (错误码 {result.returncode})")
                print("  请检查上面的错误信息，确认串口号是否正确。")
        except KeyboardInterrupt:
            print("\n已停止.")

    except Exception as e:
        print(f"\n⚠ 启动器发生异常: {e}")
        import traceback
        traceback.print_exc()

    input("\n按回车退出...")


if __name__ == "__main__":
    main()
