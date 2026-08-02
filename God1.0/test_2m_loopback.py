# -*- coding: utf-8 -*-
"""
2Mbps 回环丢帧测试 (CH343 TX 短接 RX, 不接 FPGA)
=================================================
用法:
    py test_2m_loopback.py                # 自动检测 CH343 端口
    py test_2m_loopback.py COM7           # 指定端口
    py test_2m_loopback.py COM7 50        # 指定端口 + 发送帧数

原理:
    发送带帧计数器的合成帧 (AA 55 AA 55 + 19200B内容 + 8B结果(含帧号) + 55 AA 55 AA),
    回环回来后逐帧校验, 统计:
      - 发送/接收字节数            → 丢字节率
      - 收到的有效帧数 vs 发送帧数 → 丢帧率
      - 每帧帧号是否连续           → 帧是否丢(即便字节没丢)
      - 内容采样是否一致           → 静默错字
    只有一条线 TX 短接 RX, 发出的每一个字节都会被自己收回来.
"""
import sys, time, serial, struct
from serial.tools import list_ports

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HEADER  = b"\xAA\x55\xAA\x55"
FOOTER  = b"\x55\xAA\x55\xAA"
IMG_LEN   = 19200
MEAS_LEN  = 8
FRAME_LEN = 4 + IMG_LEN + MEAS_LEN + 4   # 19216

def build_frame(counter):
    """合成一帧: 内容每字节 = (i*3+counter)&0xFF, 帧号放在测量字节 gap_pix 字段."""
    payload = bytes(((i * 3) + counter) & 0xFF for i in range(IMG_LEN))
    meas = struct.pack(">H", counter) + b"\x00\x00\x00\x01\xFF\xFF"
    return HEADER + payload + meas + FOOTER

def verify_frame(frame, counter):
    """返回 'ok' 或错误原因."""
    if len(frame) != FRAME_LEN:
        return "len"
    if frame[0:4] != HEADER:
        return "header"
    if frame[FRAME_LEN-4:FRAME_LEN] != FOOTER:
        return "footer"
    got = struct.unpack(">H", frame[4+IMG_LEN:4+IMG_LEN+2])[0]
    if got != counter:
        return f"counter({got}!=>{counter})"
    for idx in (0, 1, 100, 1000, IMG_LEN//2, IMG_LEN-100, IMG_LEN-1):
        if frame[4+idx] != ((idx * 3 + counter) & 0xFF):
            return "payload"
    return "ok"

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
    n_frames = 50
    if len(sys.argv) >= 2:
        port = sys.argv[1]
    if len(sys.argv) >= 3:
        n_frames = int(sys.argv[2])
    if port is None:
        port = pick_port()
        if not port:
            sys.exit(1)
        print(f"自动选择端口: {port}")
    else:
        print(f"使用端口: {port}")

    try:
        ser = serial.Serial(port, 2000000, bytesize=8, parity='N',
                            stopbits=1, timeout=1.0)
    except serial.SerialException as e:
        print(f"[错误] 打不开 {port}: {e}\n  请确认没有其他程序占用, 且 TX/RX 已短接")
        sys.exit(1)

    ser.reset_input_buffer()
    print(f"已连接 {port} @ 2000000, 发送 {n_frames} 帧回环测试...\n")

    # 预生成所有帧
    frames = [build_frame(i) for i in range(n_frames)]
    send_bytes = b"".join(frames)

    t0 = time.time()
    ser.write(send_bytes)
    ser.flush()
    write_time = time.time() - t0

    # 读取回环数据 (期望收到同样字节数)
    echo = bytearray()
    t0 = time.time()
    expected = len(send_bytes)
    timeout_s = expected / (2000000/10) * 2 + 5.0   # 发送时间×2 + 余量
    while len(echo) < expected and time.time() - t0 < timeout_s:
        chunk = ser.read(65536)
        if chunk:
            echo.extend(chunk)
    read_time = time.time() - t0

    got_bytes = len(echo)
    print(f"发送: {len(send_bytes)} 字节 ({n_frames} 帧), 耗时 {write_time*1000:.0f} ms")
    print(f"接收: {got_bytes} 字节, 耗时 {read_time*1000:.0f} ms")
    print(f"字节丢失: {len(send_bytes)-got_bytes}  ({(len(send_bytes)-got_bytes)*100/max(len(send_bytes),1):.3f}%)\n")

    # 逐帧解析回环数据
    buf = echo
    result = {"ok": 0, "len": 0, "header": 0, "footer": 0, "counter": 0, "payload": 0}
    found = []
    pos = 0
    while True:
        hi = bytes(buf).find(HEADER, pos)
        if hi < 0:
            break
        if hi + FRAME_LEN <= len(buf):
            frame = bytes(buf[hi:hi+FRAME_LEN])
            # 帧号 = 测量字节里的 counter
            counter = struct.unpack(">H", frame[4+IMG_LEN:4+IMG_LEN+2])[0]
            r = verify_frame(frame, counter)
            result[r if r == "ok" else ("counter" if r.startswith("counter") else r)] += 1
            found.append(counter)
            pos = hi + FRAME_LEN
        else:
            break

    print("=== 逐帧校验结果 ===")
    for k, v in result.items():
        print(f"  {k:>8}: {v}")
    if result["ok"] == n_frames:
        print("\n判定: OK - 所有帧完整回环, 无丢帧/丢字节/错字, 2M 链路干净")
    elif result["ok"] == 0:
        print("\n判定: 一帧都没收到 - 检查 TX/RX 是否真的短接、端口是否选对")
    else:
        dropped = n_frames - result["ok"]
        print(f"\n判定: 有 {dropped}/{n_frames} 帧丢失或损坏 - 2M 链路有丢数据")

    # 检查帧号连续性 (即便所有帧都 ok, 也确认没有静默跳号)
    if found:
        expected_seq = list(range(n_frames))
        missing = sorted(set(expected_seq) - set(found))
        if missing:
            print(f"  帧号缺口: {missing[:20]}{'...' if len(missing)>20 else ''} (共 {len(missing)} 帧)")

    ser.close()

if __name__ == "__main__":
    main()
