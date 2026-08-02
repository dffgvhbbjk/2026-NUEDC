# -*- coding: utf-8 -*-
"""
抓取当前 2M 串口的帧, 保存原始图像数据 + 统计帧率/帧间隔.
用法:
    py capture_frames.py COM7        # 抓 10 秒, 存 frame0.bin / frame1.bin / ...
    py capture_frames.py COM7 15     # 抓 15 秒
"""
import sys, time, serial, struct

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HEADER  = b"\xAA\x55\xAA\x55"
FOOTER  = b"\x55\xAA\x55\xAA"
FRAME_LEN = 19216
IMG_LEN   = 19200

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    try:
        ser = serial.Serial(port, 2000000, bytesize=8, parity='N',
                            stopbits=1, timeout=0.05)
    except serial.SerialException as e:
        print(f"[错误] 打不开 {port}: {e}\n  先关闭 Qt 上位机/其他占用该串口的程序")
        sys.exit(1)
    ser.reset_input_buffer()

    buf = bytearray()
    frames = []
    gaps = []
    last_t = None
    t0 = time.time()
    print(f"抓取 {dur}s, 保存前几个有效帧...")
    while time.time() - t0 < dur:
        chunk = ser.read(65536)
        if chunk:
            buf.extend(chunk)
        while True:
            hi = buf.find(HEADER)
            if hi < 0:
                if len(buf) > 3:
                    buf = buf[-3:]
                break
            if hi > 0:
                buf = buf[hi:]
            if len(buf) < FRAME_LEN:
                break
            if buf[FRAME_LEN-4:FRAME_LEN] == FOOTER:
                now = time.time()
                if last_t is not None:
                    gaps.append((now - last_t) * 1000.0)
                last_t = now
                frame = bytes(buf[:FRAME_LEN])
                frames.append(frame)
                if len(frames) <= 3:
                    fn = f"frame{len(frames)-1}.bin"
                    with open(fn, "wb") as f:
                        f.write(frame)
                    # 解析测量字节
                    meas = frame[4+IMG_LEN:4+IMG_LEN+8]
                    print(f"  已存 {fn}: gap_pix={int.from_bytes(meas[0:2],'big')} "
                          f"mm_x10={int.from_bytes(meas[2:4],'big')} "
                          f"status=0x{meas[5]:02x}")
                buf = buf[FRAME_LEN:]
            else:
                buf = buf[1:]
        # 简单进度
        if int(time.time() - t0) % 3 == 0 and frames:
            pass

    ser.close()
    elapsed = time.time() - t0
    print(f"\n共收到 {len(frames)} 帧, 平均 {len(frames)/elapsed:.2f} fps")
    if gaps:
        gaps.sort()
        g = gaps
        print(f"帧间隔: min={g[0]:.0f}ms  中位={g[len(g)//2]:.0f}ms  "
              f"p90={g[int(len(g)*0.9)]:.0f}ms  max={g[-1]:.0f}ms")
        # 检测异常大间隔 (丢帧信号)
        big = [x for x in gaps if x > g[len(g)//2] * 1.8]
        if big:
            print(f"⚠ 有 {len(big)} 个帧间隔明显偏大(可能丢帧): "
                  f"最大 {max(big):.0f}ms")
    # 帧内容检查
    if frames:
        img = frames[0][4:4+IMG_LEN]
        nz = sum(1 for b in img if b != 0)
        print(f"首帧图像: 非零 {nz}/{IMG_LEN}, min={min(img)} max={max(img)} "
              f"mean={sum(img)/len(img):.1f}")
        print(f"已保存 frame0.bin 等, 可用 analyze 脚本分析花屏结构")

if __name__ == "__main__":
    main()
