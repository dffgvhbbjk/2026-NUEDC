# -*- coding: utf-8 -*-
"""
2Mbps 串口链路丢帧测试
======================
用法:
    py test_2m_link.py                # 自动检测 CH343 端口
    py test_2m_link.py COM5           # 指定端口
    py test_2m_link.py COM5 2000000   # 指定端口和波特率

行为:
    - 打开串口(默认 2M baud, 8N1)
    - 流式解析 19216 字节帧 (AA 55 AA 55 + 19200B图像 + 8B结果 + 55 AA 55 AA)
    - 每秒打印: 接收字节率 / 有效帧率 / 帧尾不匹配(丢字节) / 重同步次数
    - Ctrl+C 结束, 打印汇总: 总帧数 / 平均帧率 / 丢帧率

判读:
    - 若持续出现 footer_mismatch > 0 → 链路上丢字节(2M 不稳)
    - 若有效帧率稳定且帧间间隔均匀 → 链路干净
    - 若帧间出现异常大间隔(>2倍平均) → 中间有帧被丢
"""
import sys, time, serial
from serial.tools import list_ports

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HEADER  = b"\xAA\x55\xAA\x55"
FOOTER  = b"\x55\xAA\x55\xAA"
FRAME_LEN = 19216   # 4 + 19200 + 8 + 4
IMG_LEN   = 19200
MEAS_LEN  = 8

def pick_port():
    """自动选 CH343 端口, 否则列出所有端口让用户选."""
    ports = list_ports.comports()
    for p in ports:
        name = (p.description or "").lower()
        if "ch343" in name or "ch342" in name or "wch" in name or "usb-serial" in name:
            return p.device, p.description
    if ports:
        print("没找到 CH343, 可用端口:")
        for p in ports:
            print(f"   {p.device}  {p.description}")
        return None, None
    print("没有检测到任何串口!")
    return None, None

def main():
    port = None
    baud = 2000000
    if len(sys.argv) >= 2:
        port = sys.argv[1]
    if len(sys.argv) >= 3:
        baud = int(sys.argv[2])

    if port is None:
        port, desc = pick_port()
        if not port:
            sys.exit(1)
        print(f"自动选择端口: {port} ({desc})")
    else:
        print(f"使用端口: {port}")

    ser = None
    try:
        ser = serial.Serial(port, baud, bytesize=8, parity='N', stopbits=1,
                            timeout=0.1)
    except serial.SerialException as e:
        msg = str(e)
        if "PermissionError" in msg or "拒绝访问" in msg or "Access" in msg:
            print(f"\n[错误] 打不开 {port}: 端口被其他程序占用!")
            print("  请先关闭串口助手 / Qt 上位机 / 其他占用该串口的程序, 再重试。")
        else:
            print(f"\n[错误] 打不开 {port}: {msg}")
        sys.exit(1)
    ser.reset_input_buffer()
    print(f"已连接 {port} @ {baud} baud (8N1)")
    print("正在接收... Ctrl+C 停止\n")

    buf = bytearray()
    stats = {
        "bytes": 0,            # 本次周期内接收字节
        "valid": 0,            # 有效帧 (帧头帧尾都对)
        "mismatch": 0,         # 帧头找到但帧尾不对 = 该窗口丢字节
        "sync": 0,             # 重同步次数 (跳过多余字节)
        "fps_ok": True,
    }
    total = {"valid": 0, "mismatch": 0, "bytes": 0}
    t0 = time.time()
    last_report = t0
    last_frame_t = None
    max_gap = 0.0

    print(f"{'时间(s)':>8} {'KB/s':>7} {'fps':>5} {'有效帧':>7} "
          f"{'帧尾错':>6} {'重同步':>6}  帧间隔最大(ms)")
    try:
        while True:
            chunk = ser.read(65536)
            if chunk:
                buf.extend(chunk)
                stats["bytes"] += len(chunk)

            # 逐帧解析
            while True:
                hi = buf.find(HEADER)
                if hi < 0:
                    if len(buf) > 3:
                        buf = buf[-3:]      # 保留可能是帧头前缀的字节
                    break
                if hi > 0:
                    buf = buf[hi:]          # 跳过多余字节, 对齐帧头
                    stats["sync"] += 1
                if len(buf) < FRAME_LEN:
                    break                   # 数据不足, 等更多
                if buf[FRAME_LEN-4:FRAME_LEN] == FOOTER:
                    stats["valid"] += 1
                    total["valid"] += 1
                    now = time.time()
                    if last_frame_t is not None:
                        gap = (now - last_frame_t) * 1000.0
                        if gap > max_gap:
                            max_gap = gap
                    last_frame_t = now
                    buf = buf[FRAME_LEN:]
                else:
                    stats["mismatch"] += 1
                    total["mismatch"] += 1
                    buf = buf[1:]           # 伪帧头, 丢 1 字节继续

            # 每秒报告
            now = time.time()
            if now - last_report >= 1.0:
                dt = now - last_report
                kbps = stats["bytes"] / dt / 1024.0
                fps = stats["valid"] / dt
                print(f"{now-t0:>8.1f} {kbps:>7.1f} {fps:>5.1f} "
                      f"{total['valid']:>7} {total['mismatch']:>6} "
                      f"{stats['sync']:>6}  {max_gap:>6.0f}")
                # 周期清零 (sync/bytes 只关心新产生的)
                stats["bytes"] = 0
                stats["valid"] = 0
                stats["sync"] = 0
                last_report = now

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    elapsed = time.time() - t0
    print("\n========== 汇总 ==========")
    print(f"总时长: {elapsed:.1f}s")
    print(f"总字节: {total['bytes']}  ({total['bytes']/1024.0:.1f} KB)")
    if total["valid"]:
        print(f"有效帧: {total['valid']}  平均 {total['valid']/elapsed:.2f} fps")
        print(f"最大帧间隔: {max_gap:.0f} ms  (期望 ~125ms)")
    print(f"帧尾不匹配(丢字节): {total['mismatch']}")
    if total["mismatch"] == 0 and total["valid"] > 0:
        print("\n判定: OK - 链路干净, 没有丢字节, 帧全部对齐")
    elif total["valid"] == 0:
        print("\n判定: 没有解析到任何有效帧 - 检查波特率/接线/FPGA固件")
    else:
        print("\n判定: 有丢字节 - 2M 链路不稳, 需要排查")

if __name__ == "__main__":
    main()
