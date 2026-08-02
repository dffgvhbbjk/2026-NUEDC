# -*- coding: utf-8 -*-
# 自测: 构造含假帧头/垃圾/截断残帧的字节流, 验证 uart_frame_parser 防误同步
# 以及 run_file 端到端输出 (hit CSV / summary.csv 真值与误差列)
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import csv
import os
import shutil
import sys
import random
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capture.uart_frame_parser import try_parse, run_file, LEN_TABLE, RESULT_VER


def build_frame(samples, state=2, conf=2, ready=1,
                imp=3, dfc=40, bot=86, dist=470, thr=45840):
    n = len(samples)
    len_sel = {64: 0, 128: 1, 256: 2, 512: 3}[n]
    data = bytearray()
    for v in samples:
        u = v & 0xFFFFFF
        data += bytes([(u >> 16) & 0xFF, (u >> 8) & 0xFF, u & 0xFF])
    chksum = (len_sel + sum(data)) & 0xFF
    idx_h = ((bot >> 8) & 1) << 2 | ((dfc >> 8) & 1) << 1 | ((imp >> 8) & 1)
    status = (ready << 4) | (conf << 2) | state
    body = bytes([RESULT_VER, status, idx_h, imp & 0xFF, dfc & 0xFF,
                  bot & 0xFF, (dist >> 8) & 0xF, dist & 0xFF,
                  (thr >> 16) & 0xFF, (thr >> 8) & 0xFF, thr & 0xFF])
    rsum = sum(body) & 0xFF
    return bytes([0xAA, 0x55, len_sel]) + bytes(data) + bytes([chksum]) \
        + bytes([0x5A]) + body + bytes([rsum])


def main():
    random.seed(42)
    err = 0

    # 载荷: 256 点, 其中故意埋 4 处 AA55 序列 (样点 0xAA55xx / xxAA55)
    # (也可用 512 测试, 但 256 点已足够覆盖防误同步逻辑)
    samples = [random.randint(-30000, 30000) for _ in range(256)]
    samples[10] = -5614933          # 0xAA55AB: 字节 AA 55 AB
    samples[11] = 0x00AA55          #           字节 00 AA 55
    samples[100] = -5614934         # 再埋两处
    samples[101] = 0x55AA55
    good = build_frame(samples)
    assert good.count(b"\xAA\x55") >= 4, "测试流必须含多个假帧头"

    # 流 = 垃圾 + 帧1 + 截断残帧 + 帧2 + 垃圾
    frame2 = build_frame([i * 100 - 12800 for i in range(256)],
                         state=1, conf=1, dfc=0, dist=0)
    stream = (bytes([0x12, 0xAA, 0x55, 0x02, 0x99])   # 垃圾含假头
              + good
              + good[:200]                             # 截断残帧
              + frame2
              + bytes([0xAA, 0x55]))                   # 尾部悬挂帧头

    # 逐帧解析
    got = []
    pos = 0
    dropped = 0
    while pos < len(stream):
        frame, consumed = try_parse(stream[pos:])
        if frame is not None:
            got.append(frame)
            pos += consumed
        elif consumed > 0:
            dropped += consumed
            pos += consumed
        else:
            break

    print("解析到 %d 帧, 丢弃 %d 字节, 尾部残留 %d 字节"
          % (len(got), dropped, len(stream) - pos))

    if len(got) != 2:
        print("[FAIL] 期望 2 帧, 得到 %d" % len(got)); err += 1
    else:
        f1, f2 = got
        if f1["samples"] != samples:
            print("[FAIL] 帧1 样点不一致"); err += 1
        if f1["samples"][10] != -5614933 or f1["samples"][11] != 0x00AA55:
            print("[FAIL] 帧1 埋点样点解码错误"); err += 1
        if (f1["state"], f1["confidence"], f1["distance_mm"],
                f1["threshold"]) != (2, 2, 470, 45840):
            print("[FAIL] 帧1 结果段错误: %s" % f1); err += 1
        if (f1["impact_index"], f1["defect_index"],
                f1["bottom_index"]) != (3, 40, 86):
            print("[FAIL] 帧1 索引错误"); err += 1
        if f2["state"] != 1 or f2["samples"][0] != -12800:
            print("[FAIL] 帧2 错误"); err += 1

    # 单独测: 篡改 1 个数据字节 -> 必须整帧拒收
    bad = bytearray(good)
    bad[50] ^= 0x01
    frame, consumed = try_parse(bytes(bad))
    if frame is not None:
        print("[FAIL] 篡改数据字节仍被接收"); err += 1
    # 篡改 rsum -> 拒收
    bad2 = bytearray(good)
    bad2[-1] ^= 0xFF
    frame, _ = try_parse(bytes(bad2))
    if frame is not None:
        print("[FAIL] 篡改 rsum 仍被接收"); err += 1

    # ---- 端到端: run_file 输出 CSV + 真值/误差列 ----
    tmpdir = tempfile.mkdtemp(prefix="uart_e2e_")
    try:
        binpath = os.path.join(tmpdir, "capture.bin")
        with open(binpath, "wb") as f:
            f.write(stream)
        outdir = os.path.join(tmpdir, "data")
        os.makedirs(outdir)
        # 真值: 缺陷 460mm, FPGA 报 470mm -> err = +10
        run_file(binpath, outdir, "缺陷棒A 中敲", {"rod_len": 998, "defect_mm": 460})

        hits = sorted(x for x in os.listdir(outdir) if x.startswith("hit_"))
        if len(hits) != 2:
            print("[FAIL] e2e 期望 2 个 hit 文件, 得到 %d" % len(hits)); err += 1
        sum_path = os.path.join(outdir, "summary.csv")
        with open(sum_path, encoding="utf-8-sig") as f:
            rows = list(csv.reader(f))
        hdr = rows[0]
        for col in ("rod_len_mm", "defect_true_mm", "err_mm", "note"):
            if col not in hdr:
                print("[FAIL] summary 缺列 %s" % col); err += 1
        if err == 0:
            r1 = dict(zip(hdr, rows[1]))   # 帧1: DEFECT dist=470
            r2 = dict(zip(hdr, rows[2]))   # 帧2: NORMAL dist=0
            if (r1["state"], r1["dist_mm"], r1["rod_len_mm"],
                    r1["defect_true_mm"], r1["err_mm"]) != \
                    ("DEFECT", "470", "998", "460", "10"):
                print("[FAIL] e2e 帧1 汇总行错误: %s" % r1); err += 1
            if r2["state"] != "NORMAL" or r2["err_mm"] != "":
                print("[FAIL] e2e 帧2 应无误差列: %s" % r2); err += 1
        # hit CSV 头部真值行
        with open(os.path.join(outdir, hits[0]), encoding="utf-8-sig") as f:
            head = f.read(400)
        if "defect_true_mm,460" not in head or "err_mm,10" not in head:
            print("[FAIL] hit CSV 头部缺真值/误差"); err += 1
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)

    print("=== self test: %s ===" % ("ALL PASS" if err == 0 else "%d FAIL" % err))
    sys.exit(1 if err else 0)


if __name__ == "__main__":
    main()
