# -*- coding: utf-8 -*-
"""
抓取多帧, 用阈值区分 花屏(乱序) 与 噪声.
用法: py analyze_many_frames.py COM7 [秒数]
"""
import sys, time, serial

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HEADER  = b"\xAA\x55\xAA\x55"
FOOTER  = b"\x55\xAA\x55\xAA"
FRAME_LEN = 4816
IMG_LEN   = 4800
W, H = 80, 60

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
    try:
        ser = serial.Serial(port, 2000000, bytesize=8, parity='N',
                            stopbits=1, timeout=0.05)
    except serial.SerialException as e:
        print(f"[错误] 打不开 {port}: {e}\n  先关闭 Qt 上位机")
        sys.exit(1)
    ser.reset_input_buffer()

    buf = bytearray()
    frames = []
    t0 = time.time()
    while time.time() - t0 < dur:
        chunk = ser.read(65536)
        if chunk:
            buf.extend(chunk)
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
    print(f"抓到 {len(frames)} 帧\n")

    imgs = [f[4:4+IMG_LEN] for f in frames]
    if not imgs:
        print("无帧!"); return

    # 1) 每帧整体均值 (看内容稳定否)
    means = [sum(img)//IMG_LEN for img in imgs]
    print(f"每帧均值: min={min(means)} max={max(means)} 前10帧={means[:10]}")

    # 2) 阈值化帧间差异 (关键: 区分噪声 vs 乱序)
    print("\n=== 连续帧差异 (阈值 10/30/60) ===")
    for i in range(min(len(imgs)-1, 12)):
        a, b = imgs[i], imgs[i+1]
        d10 = sum(1 for x,y in zip(a,b) if abs(x-y) > 10)
        d30 = sum(1 for x,y in zip(a,b) if abs(x-y) > 30)
        d60 = sum(1 for x,y in zip(a,b) if abs(x-y) > 60)
        print(f"f{i}->f{i+1}: 差>10: {d10*100//IMG_LEN}%  差>30: {d30*100//IMG_LEN}%  差>60: {d60*100//IMG_LEN}%")

    # 3) 渲染前3帧看是否连贯
    print("\n=== 帧0/1/2 ASCII (12行x16列, 每点10x10均值) ===")
    for i in range(3):
        img = imgs[i]
        print(f"--- frame{i} (mean={means[i]}) ---")
        for y in range(0, H, 10):
            s = ''
            for x in range(0, W, 10):
                v = sum(img[(y+dy)*W+x+dx] for dy in range(10) for dx in range(min(10,W-x))) // (10*min(10,W-x))
                s += '#' if v<64 else ('+' if v<128 else ('-' if v<192 else '.'))
            print('  ' + s)

    # 4) 保存 PNG
    try:
        from PIL import Image
        for i in range(0, min(len(imgs), 6)):
            Image.frombytes('L',(W,H),imgs[i]).transpose(Image.FLIP_TOP_BOTTOM).resize((W*2,H*2), Image.BILINEAR).save(f'f{i}.png')
        print("\n已保存 f0..f5.png")
    except ImportError:
        pass

if __name__ == "__main__":
    main()
