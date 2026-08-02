import re

with open(r'd:\fpgawork\God1.0\God1.0\shujv.txt', 'r', encoding='utf-8') as f:
    raw = f.read()

bytes_arr = [int(x, 16) for x in re.findall(r'0x([0-9A-Fa-f]{2})', raw)]

print(f'Total bytes: {len(bytes_arr)}')
print()

# Frame 1 boundary
print('=== Frame 1 boundary (index 4800-4820) ===')
for i in range(4800, min(4820, len(bytes_arr))):
    print(f'  [{i}] = 0x{bytes_arr[i]:02X}')

print()
print('=== Frame 2 boundary (index 9610-9627) ===')
for i in range(9610, len(bytes_arr)):
    print(f'  [{i}] = 0x{bytes_arr[i]:02X}')

# Frame analysis
print()
print('=== Frame Analysis ===')
f1_len = 4815 - 0
f2_len = 9627 - 4815
print(f'Frame 1 length: {f1_len} (expected 4816, missing {4816 - f1_len})')
print(f'Frame 2 length: {f2_len} (expected 4816, missing {4816 - f2_len})')

# Frame 1 footer
print()
print('=== Frame 1 footer check ===')
print(f'Bytes at 4808-4815: {" ".join(f"{bytes_arr[i]:02X}" for i in range(4808, 4816))}')
print(f'Footer (55 AA 55 AA) found at 4811, expected at 4812 -> 1 byte early')

# Frame 2 measurement results
print()
print('=== Frame 2 last 16 bytes ===')
print(f'Last 16 bytes: {" ".join(f"{bytes_arr[i]:02X}" for i in range(9611, 9627))}')
print(f'Expected results position: 9619-9626 (8 bytes)')
print(f'Expected footer position: 9627-9630 (4 bytes)')
print(f'Actual data ends at: 9626')
print(f'Footer (55 AA 55 AA) found at 9623, expected at 9627 -> 4 bytes early')

# Check what is at expected position for frame 2 footer
print()
print('=== Diagnosis ===')
print('Frame 1: 1 byte short -> image data missing 1 byte')
print('Frame 2: 4 bytes short -> image+results truncated (capture stopped mid-frame?)')

# Look at frame 1 measurement results assuming footer is at 4811
print()
print('=== Frame 1 measurement (assuming footer at 4811) ===')
# results = 4803 to 4811 (8 bytes)
results = bytes_arr[4803:4811]
print(f'Results bytes: {" ".join(f"{b:02X}" for b in results)}')
print(f'  gap_pix_hi    = 0x{results[0]:02X}')
print(f'  gap_pix_lo    = 0x{results[1]:02X}')
print(f'  gap_mm_hi     = 0x{results[2]:02X}')
print(f'  gap_mm_lo     = 0x{results[3]:02X}')
print(f'  detect_row_lo = 0x{results[4]:02X}')
print(f'  status        = 0x{results[5]:02X}')
print(f'  gap_left      = 0x{results[6]:02X}')
print(f'  gap_right     = 0x{results[7]:02X}')

# Look at frame 2 measurement assuming footer is at 9623
print()
print('=== Frame 2 measurement (assuming footer at 9623) ===')
# Frame 2 starts at 4815
# If footer is at 9623, results = 9615 to 9623 (8 bytes)
results2 = bytes_arr[9615:9623]
print(f'Results bytes: {" ".join(f"{b:02X}" for b in results2)}')
print(f'  gap_pix_hi    = 0x{results2[0]:02X}')
print(f'  gap_pix_lo    = 0x{results2[1]:02X}')
print(f'  gap_mm_hi     = 0x{results2[2]:02X}')
print(f'  gap_mm_lo     = 0x{results2[3]:02X}')
print(f'  detect_row_lo = 0x{results2[4]:02X}')
print(f'  status        = 0x{results2[5]:02X}')
print(f'  gap_left      = 0x{results2[6]:02X}')
print(f'  gap_right     = 0x{results2[7]:02X}')

# Check image bytes at start of frame 1
print()
print('=== Image content check ===')
# Frame 1 image at 4-4803
non_zero = [(i, bytes_arr[i]) for i in range(4, 4804) if bytes_arr[i] != 0]
print(f'Frame 1 image: {len(non_zero)} non-zero bytes out of 4800')
# Print first 10 non-zero
for idx, (i, v) in enumerate(non_zero[:10]):
    print(f'  [{i}] = 0x{v:02X}')
print(f'  ...')
for idx, (i, v) in enumerate(non_zero[-5:]):
    print(f'  [{i}] = 0x{v:02X}')

# Frame 2 image at 4819-9618 (if full frame)
# But actual data only goes to 9622 (before footer at 9623)
# So image + results = 9623 - 4819 = 4804 bytes
# That's 4 bytes short of 4808 (4800 image + 8 results)
print()
print(f'Frame 2 image+results actual length: {9623 - 4819} bytes (expected 4808)')
