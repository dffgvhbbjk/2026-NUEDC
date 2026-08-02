#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析 shujv.txt 串口数据, 判断图像花屏原因"""

import sys
from collections import Counter

path = r'D:\fpgawork\God1.0\God1.0\shujv.txt'

with open(path, 'rb') as f:
    data = f.read()

print(f"Total size: {len(data)} bytes")

# 帧格式: AA 55 AA 55 + 4800 image + 8 measure + 55 AA 55 AA = 4816 bytes
FRAME_LEN = 4816
HEADER = b'\xAA\x55\xAA\x55'
FOOTER = b'\x55\xAA\x55\xAA'

# 找第一个帧头
idx = data.find(HEADER)
print(f"First header at offset: {idx}")

if idx < 0:
    print("No header found!")
    sys.exit(1)

# 检查帧尾
foot_idx = idx + FRAME_LEN - 4
print(f"Expected footer at offset: {foot_idx}")
print(f"Footer bytes: {data[foot_idx:foot_idx+4].hex(' ')}")
print(f"Footer match: {data[foot_idx:foot_idx+4] == FOOTER}")

# 提取图像数据
img_start = idx + 4
img_end = img_start + 4800
img = data[img_start:img_end]

# 统计
print(f"\n=== Image data stats ===")
print(f"Size: {len(img)}")
print(f"Min: {min(img)}, Max: {max(img)}")
print(f"Mean: {sum(img)/len(img):.2f}")
nz = sum(1 for b in img if b != 0)
print(f"Non-zero: {nz}/{len(img)}")

# 字节值分布 (直方图, 按16分组)
print(f"\n=== Byte value histogram (16 bins) ===")
hist = Counter(img)
for i in range(0, 256, 16):
    cnt = sum(hist.get(j, 0) for j in range(i, i+16))
    bar = '#' * (cnt * 50 // len(img))
    print(f"  {i:3d}-{i+15:3d}: {cnt:5d} ({cnt*100/len(img):5.1f}%) {bar}")

# 检查奇偶字节的分布 (YUV422 YUYV: 偶数位=Y, 奇数位=U/V)
print(f"\n=== Even vs Odd byte position stats (YUV422 byte alternation) ===")
even_bytes = img[0::2]  # 假设的 Y 字节
odd_bytes = img[1::2]   # 假设的 U/V 字节
print(f"Even bytes (pos 0,2,4,...): min={min(even_bytes)} max={max(even_bytes)} mean={sum(even_bytes)/len(even_bytes):.2f}")
print(f"Odd  bytes (pos 1,3,5,...): min={min(odd_bytes)} max={max(odd_bytes)} mean={sum(odd_bytes)/len(odd_bytes):.2f}")

# YUV422 中, Y 应该有较宽的分布 (反映亮度), U/V 通常集中在 128 附近 (中性色)
# 如果采集错位, 偶数位采的是 U/V, 则分布会集中
print(f"\n=== Even byte histogram (should be Y if format is YUYV) ===")
hist_e = Counter(even_bytes)
for i in range(0, 256, 16):
    cnt = sum(hist_e.get(j, 0) for j in range(i, i+16))
    bar = '#' * (cnt * 50 // len(even_bytes))
    print(f"  {i:3d}-{i+15:3d}: {cnt:5d} ({cnt*100/len(even_bytes):5.1f}%) {bar}")

print(f"\n=== Odd byte histogram (should be U/V if format is YUYV) ===")
hist_o = Counter(odd_bytes)
for i in range(0, 256, 16):
    cnt = sum(hist_o.get(j, 0) for j in range(i, i+16))
    bar = '#' * (cnt * 50 // len(odd_bytes))
    print(f"  {i:3d}-{i+15:3d}: {cnt:5d} ({cnt*100/len(odd_bytes):5.1f}%) {bar}")

# 测量结果 8 字节
meas = data[img_end:img_end+8]
print(f"\n=== Measure pack (8 bytes) ===")
print(f"Hex: {meas.hex(' ')}")
print(f"gap_pix = 0x{meas[0]:02X}{meas[1]:02X} = {(meas[0]<<8)|meas[1]}")
print(f"gap_mm_x10 = 0x{meas[2]:02X}{meas[3]:02X} = {(meas[2]<<8)|meas[3]}")
print(f"detect_row_lo = 0x{meas[4]:02X}")
print(f"status = 0x{meas[5]:02X}  valid={(meas[5]&1)} stable={(meas[5]>>1)&1} v2={(meas[5]>>4)&1}")
print(f"gap_left = 0x{meas[6]:02X}")
print(f"gap_right = 0x{meas[7]:02X}")

# 输出前 64 字节图像数据
print(f"\n=== First 64 image bytes (hex) ===")
print(' '.join(f'{b:02X}' for b in img[:64]))

# 输出 8x6 ASCII art (与 QT 一致)
print(f"\n=== ASCII art (8x6 grid, every 10th pixel) ===")
for y in range(0, 60, 10):
    line = ""
    for x in range(0, 80, 10):
        v = img[y * 80 + x]
        if v == 0: line += "."
        elif v < 64: line += "-"
        elif v < 128: line += "+"
        elif v < 192: line += "*"
        else: line += "#"
    print(f"  {line}")
