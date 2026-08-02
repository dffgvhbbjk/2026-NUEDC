# -*- coding: utf-8 -*-
# ============================================================================
# uart_frame_parser.py -- wave_uart_export 帧解析器 (冻结版 §12.7 ver=0x02)
#
# 帧格式 (MSB first, 共 17 + 3N 字节, N = 64/128/256/512):
#   [0]  0xAA  同步头
#   [1]  0x55
#   [2]  len_sel        0=64点 1=128点 2=256点 3=512点
#   [3 .. 3+3N-1]       有符号 24 位原始样点 (大端)
#   [3+3N]  chksum      len_sel 与全部数据字节的 8 位累加和
#   ---- 版本化结果段 13 字节 ----
#   [+0] 0x5A  结果段标记
#   [+1] ver   0x02
#   [+2] status: bit[1:0]=result_state bit[3:2]=confidence bit4=result_ready
#   [+3] idx_h: bit0=impact[8] bit1=defect[8] bit2=bottom[8]
#   [+4] impact_index[7:0]
#   [+5] defect_index[7:0]
#   [+6] bottom_index[7:0]
#   [+7] dist[11:8]  [+8] dist[7:0]      缺陷距离 mm
#   [+9] thr[23:16] [+10] thr[15:8] [+11] thr[7:0]  触发阈值
#   [+12] rsum   ver..th0 共 11 字节的 8 位累加和
#
# 防误同步 (载荷中出现 AA 55 不会导致假帧):
#   候选帧必须同时满足: len_sel 合法 + 波形 chksum + 0x5A 标记
#   + ver==0x02 + 结果段 rsum, 全部通过才接收; 任一失败则滑动 1 字节重找。
#   串口模式下加字节间超时: 帧中断 >0.5s 丢弃残帧重新同步。
#
# 用法 (交互模式, 推荐):
#   python uart_frame_parser.py --port COM5
#   启动后按提示输入:
#     1. 本组说明 (如: 缺陷棒 缺陷30mm 中敲)
#     2. 计划采集次数 (如 10; 回车=不限, Ctrl+C 手动停)
#     3. 缺陷真实位置 mm (回车跳过; 给了就自动算误差)
#   收够次数自动停止; 换一组数据重新运行脚本即可。
#   每组数据独立存一个目录: data\序号_时间_说明\
#
# 命令行直接给参 (跳过交互提问):
#   --note "正常棒 中敲" --count 20 --defect-mm 350 --rod-len 998
# 离线文件:  python uart_frame_parser.py --file capture.bin
#
# 输出 (本组目录下):
#   hit_NNN_YYYYmmdd_HHMMSS.csv   每帧: 序号,原始样点 + 结果段字段
#   summary.csv                   一行一帧的汇总 (含峰值统计/真值/误差列)
# ============================================================================

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import argparse
import csv
import os
import sys
import time
from datetime import datetime

RESULT_VER = 0x02
LEN_TABLE = {0: 64, 1: 128, 2: 256, 3: 512}
STATE_NAME = {0: "INVALID", 1: "NORMAL", 2: "DEFECT", 3: "RSV3"}
CONF_NAME = {0: "NONE", 1: "LOW", 2: "HIGH", 3: "RSV3"}
TAIL_LEN = 13          # 5A ver status idx_h imp def bot dh dl t2 t1 t0 rsum
IDLE_TIMEOUT = 0.5     # 串口模式: 帧中断超时 (s), 帧间隔 >= 85ms 重武装期


def s24(b2, b1, b0):
    """大端 3 字节 -> 有符号 24 位整数"""
    v = (b2 << 16) | (b1 << 8) | b0
    return v - (1 << 24) if v & 0x800000 else v


def try_parse(buf):
    """
    在 buf[0:] 尝试解析一个完整帧。
    返回 (frame_dict, consumed) 或 (None, skip):
      frame_dict 非 None: 成功, consumed = 帧总长
      frame_dict 为 None: skip=0 表示数据不足需等待; skip>0 表示丢弃字节数
    """
    # 找同步头
    idx = buf.find(b"\xAA\x55")
    if idx < 0:
        # 保留最后 1 字节 (可能是被截断的 0xAA)
        return None, max(0, len(buf) - 1)
    if idx > 0:
        return None, idx          # 丢掉头之前的垃圾

    if len(buf) < 3:
        return None, 0
    len_sel = buf[2]
    if len_sel not in LEN_TABLE:
        return None, 1            # len 非法, 滑动 1 字节
    n = LEN_TABLE[len_sel]
    total = 3 + 3 * n + 1 + TAIL_LEN
    if len(buf) < total:
        return None, 0            # 等更多数据

    # 波形段校验: len_sel + 全部数据字节
    data = buf[3:3 + 3 * n]
    chksum = buf[3 + 3 * n]
    calc = (len_sel + sum(data)) & 0xFF
    if calc != chksum:
        return None, 1

    # 结果段校验
    tail = buf[3 + 3 * n + 1: total]
    if tail[0] != 0x5A:
        return None, 1
    if tail[1] != RESULT_VER:
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


def frame_stats(samples):
    """模板制作用的快速统计"""
    peak_pos = max(samples)
    peak_neg = min(samples)
    mean = sum(samples) / len(samples)
    return {
        "peak_pos": peak_pos,
        "peak_neg": peak_neg,
        "peak_abs": max(peak_pos, -peak_neg),
        "mean": round(mean, 1),
    }


def recalc_impact_index(samples, scan_len=100):
    """从原始样点重算冲击峰索引 (与分类器峰值扫描一致: 前 scan_len 点找 |x| 最大值)。

    背景: 部分固件导出的 impact_index 恒为 0 (旧版峰值扫描/字段问题), 但原始波形里
    冲击峰位置是对的。PC 用原始样点重算, 得到真实冲击位置, 供报告/距离分析使用。
    返回首次出现最大|值|的位置 (与 RTL `ps_abs > peak_mag` 严格大于语义一致)。
    """
    best_i = 0
    best_v = 0
    for i, v in enumerate(samples[:scan_len]):
        a = v if v >= 0 else -v
        if a > best_v:
            best_v = a
            best_i = i
    return best_i


def save_frame(frame, hit_no, outdir, note, truth):
    ts = datetime.now()
    name = "hit_%03d_%s.csv" % (hit_no, ts.strftime("%Y%m%d_%H%M%S"))
    path = os.path.join(outdir, name)
    st = frame_stats(frame["samples"])
    # PC 重算冲击峰 (FPGA 导出的 impact_index 在部分固件下恒为 0, 原始样点才可靠)
    imp_pc = recalc_impact_index(frame["samples"])
    # 误差 = FPGA 输出距离 - 真值 (仅 DEFECT 且给定真值时有意义)
    err_mm = ""
    if truth["defect_mm"] is not None and frame["state"] == 2:
        err_mm = frame["distance_mm"] - truth["defect_mm"]
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["# note", note])
        w.writerow(["# rod_len_mm", truth["rod_len"] if truth["rod_len"] is not None else ""])
        w.writerow(["# defect_true_mm", truth["defect_mm"] if truth["defect_mm"] is not None else ""])
        w.writerow(["# err_mm", err_mm])
        w.writerow(["# time", ts.strftime("%Y-%m-%d %H:%M:%S")])
        w.writerow(["# points", frame["n"]])
        w.writerow(["# state", STATE_NAME[frame["state"]]])
        w.writerow(["# confidence", CONF_NAME[frame["confidence"]]])
        w.writerow(["# result_ready", frame["result_ready"]])
        w.writerow(["# impact_index", frame["impact_index"]])
        w.writerow(["# impact_recalc", imp_pc])      # PC 重算冲击峰 (fpga==0 时的正确值)
        w.writerow(["# impact_match", 1 if frame["impact_index"] == imp_pc else 0])
        w.writerow(["# defect_index", frame["defect_index"]])
        w.writerow(["# bottom_index", frame["bottom_index"]])
        w.writerow(["# distance_mm", frame["distance_mm"]])
        w.writerow(["# threshold", frame["threshold"]])
        w.writerow(["# peak_abs", st["peak_abs"]])
        w.writerow(["index", "raw24"])
        for i, v in enumerate(frame["samples"]):
            w.writerow([i, v])

    # 汇总表 (追加)
    sum_path = os.path.join(outdir, "summary.csv")
    new = not os.path.exists(sum_path)
    with open(sum_path, "a", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["hit", "time", "file", "points", "state", "conf",
                        "impact", "impact_recalc", "impact_match", "defect", "bottom",
                        "dist_mm", "threshold",
                        "peak_abs", "peak_pos", "peak_neg", "mean",
                        "rod_len_mm", "defect_true_mm", "err_mm", "note"])
        w.writerow([hit_no, ts.strftime("%H:%M:%S"), name, frame["n"],
                    STATE_NAME[frame["state"]], CONF_NAME[frame["confidence"]],
                    frame["impact_index"], imp_pc,
                    1 if frame["impact_index"] == imp_pc else 0,
                    frame["defect_index"], frame["bottom_index"],
                    frame["distance_mm"],
                    frame["threshold"], st["peak_abs"], st["peak_pos"],
                    st["peak_neg"], st["mean"],
                    truth["rod_len"] if truth["rod_len"] is not None else "",
                    truth["defect_mm"] if truth["defect_mm"] is not None else "",
                    err_mm, note])
    return path, st, err_mm


def report(frame, hit_no, path, st, err_mm):
    err_txt = ("" if err_mm == "" else " err=%+dmm" % err_mm)
    imp_pc = recalc_impact_index(frame["samples"])
    imp_txt = "imp=%d" % frame["impact_index"]
    if frame["impact_index"] != imp_pc:
        imp_txt += "->%d" % imp_pc   # FPGA 报的冲击位置与 PC 重算不符 (常见: 旧固件恒为 0)
    print("[HIT %03d] %s conf=%s  %s def=%d bot=%d dist=%dmm%s thr=%d "
          "peak=%d -> %s" % (
              hit_no, STATE_NAME[frame["state"]],
              CONF_NAME[frame["confidence"]], imp_txt,
              frame["defect_index"], frame["bottom_index"],
              frame["distance_mm"], err_txt, frame["threshold"],
              st["peak_abs"], os.path.basename(path)))


def run_file(fname, outdir, note, truth):
    with open(fname, "rb") as f:
        buf = f.read()
    hit = 0
    pos = 0
    dropped = 0
    while pos < len(buf):
        frame, consumed = try_parse(buf[pos:])
        if frame is not None:
            hit += 1
            path, st, err_mm = save_frame(frame, hit, outdir, note, truth)
            report(frame, hit, path, st, err_mm)
            pos += consumed
        elif consumed > 0:
            dropped += consumed
            pos += consumed
        else:
            break                 # 尾部不完整
    tail_left = len(buf) - pos
    print("完成: %d 帧, 丢弃 %d 字节, 尾部残留 %d 字节" %
          (hit, dropped, tail_left))


def run_serial(port, baud, outdir, note, truth, count=None,
               on_frame=None, check_continue=None):
    """串口采集主循环。
    on_frame(frame_dict, hit_no): 每收到一帧并保存后回调, 返回 True 继续/False 停止。
    check_continue(): 每轮主循环调用, 返回 False 停止采集。"""
    try:
        import serial
    except ImportError:
        sys.exit("需要 pyserial: pip install pyserial")

    # Windows COM≥10 需要 \\.\ 前缀
    def _open_serial(p, b):
        try:
            return serial.Serial(p, b, timeout=0.05)
        except serial.SerialException:
            if p.upper().startswith("COM"):
                try:
                    return serial.Serial(f"\\\\.\\{p}", b, timeout=0.05)
                except serial.SerialException:
                    pass
            raise

    try:
        ser = _open_serial(port, baud)
    except serial.SerialException as e:
        print(f"\n⚠ 无法打开串口 {port}: {e}")
        print("  请检查:")
        print("  1. 设备管理器中串口号是否正确")
        print("  2. 串口是否被其他程序占用 (串口助手等)")
        try:
            from serial.tools import list_ports
            ports = list(list_ports.comports())
            if ports:
                print("\n  当前可用串口:")
                for p in ports:
                    print(f"    {p.device}  --  {p.description}")
            else:
                print("\n  未检测到任何串口! 请确认 FPGA 板已连接。")
        except Exception:
            pass
        input("\n按回车退出...")
        return
    plan = ("目标 %d 次" % count) if count else "不限次数, Ctrl+C 结束"
    print("监听 %s @ %d bps (%s), 开始敲击 ..." % (port, baud, plan))
    buf = bytearray()
    hit = 0
    dropped = 0
    done = False
    last_rx = time.monotonic()
    try:
        while not done:
            # 检查外部停止条件 (如波形窗口关闭)
            if check_continue is not None and not check_continue():
                print("\n外部停止")
                break

            chunk = ser.read(4096)
            now = time.monotonic()
            if chunk:
                buf += chunk
                last_rx = now
            elif buf and (now - last_rx) > IDLE_TIMEOUT:
                print("  [警告] 超时丢弃残帧 %d 字节" % len(buf))
                dropped += len(buf)
                buf.clear()
                continue
            # 解析缓冲区内所有可用帧
            while buf:
                frame, consumed = try_parse(bytes(buf))
                if frame is not None:
                    hit += 1
                    path, st, err_mm = save_frame(frame, hit, outdir, note, truth)
                    report(frame, hit, path, st, err_mm)
                    # 外部回调 (如 PC 判别 + 波形)
                    if on_frame is not None:
                        if not on_frame(frame, hit):
                            done = True
                            break
                    del buf[:consumed]
                    if count and hit >= count:
                        done = True
                        break
                    if count:
                        print("  ... 还差 %d 次" % (count - hit))
                elif consumed > 0:
                    dropped += consumed
                    del buf[:consumed]
                else:
                    break         # 等更多数据
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    print("\n本组完成: %d 帧, 丢弃 %d 字节" % (hit, dropped))
    print("数据目录: %s" % os.path.abspath(outdir))


def sanitize(text, maxlen=40):
    """说明文字 -> 安全的目录名片段"""
    bad = '\\/:*?"<>| '
    out = "".join(("_" if c in bad else c) for c in text.strip())
    return out[:maxlen] if out else "unnamed"


def ask_int(prompt):
    """交互输入整数, 回车=跳过(None), 非法输入重问"""
    while True:
        s = input(prompt).strip()
        if s == "":
            return None
        try:
            return int(s)
        except ValueError:
            print("  请输入整数或直接回车跳过")


def main():
    ap = argparse.ArgumentParser(description="wave_uart_export 帧解析器")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="串口号, 如 COM5")
    src.add_argument("--file", help="离线二进制文件 (串口助手原始保存)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--outdir", default="data", help="数据根目录 (默认 data)")
    ap.add_argument("--note", default=None, help="本组说明 (不给则启动时询问)")
    ap.add_argument("--count", type=int, default=None,
                    help="计划采集次数, 收够自动停 (不给则启动时询问)")
    ap.add_argument("--rod-len", type=int, default=None,
                    help="棒实际总长 mm (卷尺实量)")
    ap.add_argument("--defect-mm", type=int, default=None,
                    help="缺陷距敲击端真实距离 mm (正常棒不加)")
    args = ap.parse_args()

    # ---- 交互提问 (仅串口模式且未通过命令行给参时) ----
    note = args.note
    count = args.count
    defect_mm = args.defect_mm
    if args.port:
        if note is None:
            note = input("这次采什么? (如: 缺陷棒 缺陷30mm 中敲): ").strip()
        if count is None:
            count = ask_int("计划敲几次? (回车=不限, Ctrl+C 手动停): ")
        if defect_mm is None:
            defect_mm = ask_int("缺陷真实位置 mm? (正常棒/噪声测试直接回车): ")
    if note is None:
        note = ""

    truth = {"rod_len": args.rod_len, "defect_mm": defect_mm}

    if args.file:
        os.makedirs(args.outdir, exist_ok=True)
        run_file(args.file, args.outdir, note, truth)
    else:
        # 每组数据独立目录: data\序号_时间_说明\
        os.makedirs(args.outdir, exist_ok=True)
        seq = 1 + sum(1 for d in os.listdir(args.outdir)
                      if os.path.isdir(os.path.join(args.outdir, d)))
        sess = "%02d_%s_%s" % (seq, datetime.now().strftime("%H%M%S"),
                               sanitize(note))
        outdir = os.path.join(args.outdir, sess)
        os.makedirs(outdir)
        print("本组数据目录: %s" % outdir)
        run_serial(args.port, args.baud, outdir, note, truth, count)


if __name__ == "__main__":
    main()
