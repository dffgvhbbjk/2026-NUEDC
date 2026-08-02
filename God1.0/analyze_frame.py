import sys; sys.stdout.reconfigure(encoding="utf-8", errors="replace")
# -*- coding: utf-8 -*-
import re, sys
from collections import Counter

path = sys.argv[1] if len(sys.argv) > 1 else "c:/Users/31368/Desktop/God1.0-sxt(7)/God1.0/shujv.txt"
with open(path, 'rb') as f:
    raw = f.read()

text = raw.decode('latin-1', errors='replace')
if '0x' in text:
    toks = re.findall(r'0x([0-9a-fA-F]{2})', text)
    data = bytes(int(t, 16) for t in toks)
    print(f"Text hex -> {len(data)} bytes")
else:
    data = raw
    print(f"Raw binary: {len(data)} bytes")

print("=" * 60)
print("1) 各种标志模式出现次数:")
for name, pat in [("AA 55 AA 55", b'\xAA\x55\xAA\x55'),
                  ("55 AA 55 AA", b'\x55\xAA\x55\xAA'),
                  ("AA 55 对", b'\xAA\x55'),
                  ("55 AA 对", b'\x55\xAA'),
                  ("0x5A (指令头)", b'\x5A')]:
    print(f"   {name}: {data.count(pat)}")

print("=" * 60)
print("2) 周期性检查 (假设帧长 N, 看每隔 N 字节是否重复):")
for N in (19216, 4816, 19200, 4800):
    if N <= len(data):
        # 采样点之间的相似性: 取前几个位置, 比较 data[i] 和 data[i+N]
        match = sum(1 for i in range(0, min(len(data)-N, 2000), 1) if data[i] == data[i+N])
        tot = min(len(data)-N, 2000)
        print(f"   N={N}: 相邻N字节相同比例 = {match}/{tot} ({match*100/tot:.1f}%)")

print("=" * 60)
print("3) 字节值分布 (整体):")
hist = Counter(data)
for i in range(0, 256, 16):
    cnt = sum(hist.get(j, 0) for j in range(i, i+16))
    bar = '#' * (cnt * 40 // len(data))
    print(f"   {i:3d}-{i+15:3d}: {cnt:5d} ({cnt*100/len(data):4.1f}%) {bar}")

print("=" * 60)
print("4) 前 200 字节原始:")
print("   " + data[:200].hex())

print("=" * 60)
print("5) 数据是否像灰度图 (假设 160x120 或 80x60, 行均值波动):")
for W, H in ((160, 120), (80, 60)):
    if W*H <= len(data):
        img = data[:W*H]
        means = [sum(img[r*W:(r+1)*W])/W for r in range(min(H, 20))]
        print(f"   {W}x{H}: 前20行均值: " + " ".join(f"{m:.0f}" for m in means))
