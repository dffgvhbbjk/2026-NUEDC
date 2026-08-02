# -*- coding: utf-8 -*-
"""
OV5640 缝隙检测 上位机诊断脚本
==============================
功能:
  1. 打开串口 (默认自动找 CH343, 2M baud)
  2. 抓取 N 秒帧数据
  3. 统计: 帧率 / 帧间隔 / 帧间差异(撕裂) / 暗区位置稳定性
  4. 保存前几帧为 .bin 供进一步分析

用法:
  py gap_diag.py               # 自动检测 CH343 端口, 抓 12 秒
  py gap_diag.py COM7          # 指定端口
  py gap_diag.py COM7 15       # 指定端口 + 抓取秒数

判读:
  - fps: 预期 ~16 (若曝光限制生效) 或 ~8 (摄像头限制)
  - 帧间差异(差>30%): 高 = 撕裂/画面不稳定, 低 = 画面稳定
  - 暗区位置变化范围: 小 = 画面稳定, 大 = 左移/跳变
"""
import sys, time, serial, os
from serial.tools import list_ports

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HEADER  = b"\xAA\x55\xAA\x55"
FOOTER  = b"\x55\xAA\x55\xAA"
FRAME_LEN = 4816
IMG_LEN   = 4800
W, H = 80, 60

def pick_port():
    ports = list_ports.comports()
    for p in ports:
        if "ch343" in (p.description or "").lower():
            return p.device
    if ports:
        print("没找到 CH343, 可用端口:")
        for p in ports:
            print(f"   {p.device}  {p.description}")
        return None
    print("没有检测到任何串口!")
    return None

def main():
    port = None
    dur = 12.0
    if len(sys.argv) >= 2: port = sys.argv[1]
    if len(sys.argv) >= 3: dur = float(sys.argv[2])
    if not port:
        port = pick_port()
        if not port: sys.exit(1)
        print(f"自动选择端口: {port}")

    try:
        ser = serial.Serial(port, 2000000, bytesize=8, parity='N',
                            stopbits=1, timeout=0.02)
    except serial.SerialException as e:
        print(f"[错误] 打不开 {port}: {e}\n  先关闭 Qt 上位机/占用串口的程序")
        sys.exit(1)
    ser.reset_input_buffer()

    buf = bytearray()
    frames = []
    t0 = time.time()
    print(f"抓取 {dur}s 于 {port} @ 2M ...")
    while time.time() - t0 < dur:
        c = ser.read(65536)
        if c: buf.extend(c)
        while True:
            hi = buf.find(HEADER)
            if hi < 0:
                if len(buf) > 3: buf = buf[-3:]
                break
            if hi > 0: buf = buf[hi:]
            if len(buf) < FRAME_LEN: break
            if buf[FRAME_LEN-4:FRAME_LEN] == FOOTER:
                frames.append(bytes(buf[:FRAME_LEN]))
                buf = buf[FRAME_LEN:]
            else:
                buf = buf[1:]
    ser.close()

    n = len(frames)
    fps = n / dur
    print(f"\n========== 结果 ==========")
    print(f"抓到 {n} 帧 / {dur:.0f}s = {fps:.2f} fps")

    if n < 10:
        print("帧太少, 无法分析!")
        return

    imgs = [f[4:4+IMG_LEN] for f in frames]

    # 帧间隔
    # (需要精确时间, 从帧数估算)
    print(f"\n[帧率] {fps:.2f} fps  (目标 ~16)")

    # 帧间差异 (撕裂指标): 用阈值30
    diffs = []
    for i in range(min(n-1, 30)):
        d = sum(1 for a,b in zip(imgs[i], imgs[i+1]) if abs(a-b) > 30)
        diffs.append(d*100/IMG_LEN)
    print(f"[撕裂] 相邻帧差异>30: 平均 {sum(diffs)/len(diffs):.1f}%  "
          f"(<10%稳定, 10-30%轻微, >30%撕裂明显)")

    # 暗区位置稳定性
    def dark_cols(img):
        return [x for x in range(W) if sum(1 for y in range(H) if img[y*W+x]<100) > H//2]
    lefts = []
    for img in imgs[:40]:
        c = dark_cols(img)
        if c: lefts.append(c[0])
    if lefts:
        span = max(lefts) - min(lefts)
        print(f"[左移] 暗区左边界范围: {min(lefts)}~{max(lefts)} (跨度{span})  "
              f"(跨度<3稳定, 3-8轻微, >8明显跳变)")
    else:
        print("[左移] 40帧内没找到明显暗区")

    # 保存前3帧
    for i in range(min(3, n)):
        fn = f"diag_frame{i}.bin"
        with open(fn, "wb") as f:
            f.write(frames[i])
    print(f"\n已保存 diag_frame0..2.bin 供进一步分析")

if __name__ == "__main__":
    main()
