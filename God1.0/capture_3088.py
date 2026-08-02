# -*- coding: utf-8 -*-
"""
抓取新格式帧 (3088B: 帧头4 + 图像3072 + 结果8 + 帧尾4), 自动探测 FPGA 串口.
用法:
    py capture_3088.py                # 探测所有 COM 口, 找到 FPGA 后抓 10s
    py capture_3088.py COM7           # 指定端口抓 10s
    py capture_3088.py COM7 15        # 指定端口抓 15s
"""
import sys, time, serial
from serial.tools import list_ports

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HEADER    = b"\xAA\x55\xAA\x55"
FOOTER    = b"\x55\xAA\x55\xAA"
FRAME_LEN = 3088          # 帧头4 + 图像3072 + 结果8 + 帧尾4
IMG_LEN   = 3072
IMG_LEN_S = 4 + IMG_LEN   # 测量字节起始
W, H      = 64, 48

def open_ser(port, baud=2000000):
    return serial.Serial(port, baud, bytesize=8, parity='N', stopbits=1, timeout=0.05)

def probe(port, t=1.5):
    """开端口读 t 秒, 返回 (字节数, 找到的帧头个数, 找到的完整帧数)."""
    try:
        ser = open_ser(port)
    except Exception:
        return (None, 0, 0)
    ser.reset_input_buffer()
    buf = bytearray()
    n = 0
    t0 = time.time()
    while time.time() - t0 < t:
        c = ser.read(65536)
        if c:
            n += len(c)
            buf.extend(c)
    ser.close()
    n_hdr = buf.count(HEADER)
    # 找完整帧 (在缓冲区里数帧尾出现在帧头后 FRAME_LEN-4 处)
    n_frames = 0
    i = buf.find(HEADER)
    while i >= 0 and i + FRAME_LEN <= len(buf):
        if buf[i+FRAME_LEN-4:i+FRAME_LEN] == FOOTER:
            n_frames += 1
            i = buf.find(HEADER, i + FRAME_LEN)
        else:
            i = buf.find(HEADER, i + 1)
    return (n, n_hdr, n_frames)

def main():
    dur = 10.0
    ports = []
    for a in sys.argv[1:]:
        if a.upper().startswith("COM"):
            ports.append(a.upper())
        else:
            try:
                dur = float(a)
            except ValueError:
                pass

    if not ports:
        ports = [p.device for p in list_ports.comports()]
        print(f"探测端口: {ports}")
        results = []
        for p in ports:
            n, nh, nf = probe(p)
            if n is None:
                print(f"  {p}: 打不开 (被占用?)")
            else:
                print(f"  {p}: {n}B {nh}帧头 {nf}完整帧")
                results.append((nf, p))
        if not results:
            print("没有可用的 FPGA 端口。请先关闭 Qt/串口助手, 并确认 CH343 已连接。")
            return
        # 取完整帧最多的端口; 都没有帧则取字节最多的
        results.sort(reverse=True)
        if results[0][0] == 0:
            results.sort(key=lambda x: 1, reverse=True)  # 都无帧, 随便第一个
        port = results[0][1]
        print(f"\n选择 {port} 抓帧")
    else:
        port = ports[0]

    try:
        ser = open_ser(port)
    except Exception as e:
        print(f"[错误] 打不开 {port}: {e}\n  先关闭 Qt 上位机/占用该串口的程序")
        return
    ser.reset_input_buffer()
    buf = bytearray()
    frames = []
    gaps = []
    last_t = None
    t0 = time.time()
    print(f"在 {port} 抓取 {dur}s ...")
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
                    meas = frame[IMG_LEN_S:IMG_LEN_S+8]
                    status = meas[5]
                    print(f"  存 {fn}: gap_pix={int.from_bytes(meas[0:2],'big')} "
                          f"mm_x10={int.from_bytes(meas[2:4],'big')} "
                          f"row_lo={meas[4]:3d} status=0x{status:02x} "
                          f"(valid={status&1} stable={status&2>>1} v2={status&0x10>>4}) "
                          f"left={meas[6]:3d} right={meas[7]:3d}")
                buf = buf[FRAME_LEN:]
            else:
                buf = buf[1:]
    ser.close()
    elapsed = time.time() - t0
    print(f"\n共收到 {len(frames)} 帧, 平均 {len(frames)/elapsed:.2f} fps, "
          f"原始 {len(buf):,}B(尾部残余)")

    if frames:
        gaps.sort()
        g = gaps
        if g:
            print(f"帧间隔: min={g[0]:.0f}ms 中位={g[len(g)//2]:.0f}ms "
                  f"p90={g[int(len(g)*0.9)]:.0f}ms max={g[-1]:.0f}ms")
            big = [x for x in gaps if x > g[len(g)//2]*1.8]
            if big:
                print(f"⚠ {len(big)} 个异常大间隔 (可能丢帧), 最大 {max(big):.0f}ms")

        # 帧内容统计
        imgs = [f[4:4+IMG_LEN] for f in frames[:20]]
        means = [sum(im)//IMG_LEN for im in imgs]
        print(f"帧均值(前{len(imgs)}帧): min={min(means)} max={max(means)} "
              f"前8帧={means[:8]}")
        nz = sum(1 for b in imgs[0] if b != 0)
        print(f"首帧: 非零 {nz}/{IMG_LEN}, min={min(imgs[0])} max={max(imgs[0])} "
              f"mean={sum(imgs[0])/IMG_LEN:.1f}")

        # 帧间差异
        print("\n=== 连续帧差异 ===")
        for i in range(min(len(imgs)-1, 8)):
            a, b = imgs[i], imgs[i+1]
            d10 = sum(1 for x, y in zip(a, b) if abs(x-y) > 10)
            print(f"f{i}->f{i+1}: 差>10: {d10*100//IMG_LEN}%")

        # 行结构 (每行均值, 看是否能看到两个黑块/缝隙)
        print("\n=== 首帧行均值 (48行) ===")
        img = imgs[0]
        for y in range(H):
            row = img[y*W:(y+1)*W]
            m = sum(row)/W
            bar = '#' * int(m/16)
            print(f"  y={y:2d} mean={m:5.1f} {bar}")

        # 列结构
        print("\n=== 首帧列均值 (64列) ===")
        img = imgs[0]
        colmean = []
        for x in range(W):
            s = sum(img[y*W+x] for y in range(H))
            colmean.append(s//H)
        s = ''
        for x in range(W):
            s += '#' if colmean[x] < 64 else ('+' if colmean[x] < 128 else ('-' if colmean[x] < 192 else '.'))
        print('  列:' + s)
        print("  (暗=黑矩形, 亮=白纸/缝隙)")

if __name__ == "__main__":
    main()
