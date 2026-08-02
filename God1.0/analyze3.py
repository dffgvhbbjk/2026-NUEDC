# Python script to analyze shujv.txt
import re

with open(r"d:\fpgawork\God1.0\God1.0\shujv.txt", "r") as f:
    raw = f.read()

# extract all 0xXX hex bytes
tokens = re.findall(r'0x([0-9A-Fa-f]{2})', raw)
bytes_arr = [int(t, 16) for t in tokens]

print(f"Total bytes: {len(bytes_arr)}")
print(f"Expected frame length: 4816 (4+4800+8+4)")
print(f"Expected frame count: {len(bytes_arr) // 4816}")
print(f"Remainder bytes: {len(bytes_arr) % 4816}")
print()

# find all headers AA 55 AA 55
print("=== Header positions (AA 55 AA 55) ===")
headers = []
for i in range(len(bytes_arr) - 3):
    if bytes_arr[i] == 0xAA and bytes_arr[i+1] == 0x55 and bytes_arr[i+2] == 0xAA and bytes_arr[i+3] == 0x55:
        headers.append(i)
        print(f"  Header at index: {i}")
print(f"Total headers: {len(headers)}")
print()

# find all footers 55 AA 55 AA
print("=== Footer positions (55 AA 55 AA) ===")
footers = []
for i in range(len(bytes_arr) - 3):
    if bytes_arr[i] == 0x55 and bytes_arr[i+1] == 0xAA and bytes_arr[i+2] == 0x55 and bytes_arr[i+3] == 0xAA:
        footers.append(i)
        print(f"  Footer at index: {i}")
print(f"Total footers: {len(footers)}")
print()

# analyze each frame
print("=== Frame Analysis ===")
if not headers:
    print("No headers found!")
else:
    for f_idx, h_start in enumerate(headers):
        expected_footer_start = h_start + 4 + 4800 + 8  # 4812
        expected_frame_end = expected_footer_start + 4  # 4816

        print(f"--- Frame {f_idx + 1} ---")
        print(f"  Header start: {h_start}")
        print(f"  Expected footer start: {expected_footer_start}")
        print(f"  Expected frame end (exclusive): {expected_frame_end}")

        # header
        hb = bytes_arr[h_start:h_start+4]
        print(f"  Header bytes: " + " ".join(f"0x{b:02X}" for b in hb))

        # footer
        if expected_footer_start + 3 < len(bytes_arr):
            fb = bytes_arr[expected_footer_start:expected_footer_start+4]
            print(f"  Footer bytes at expected pos: " + " ".join(f"0x{b:02X}" for b in fb))
            footer_ok = (fb[0] == 0x55 and fb[1] == 0xAA and fb[2] == 0x55 and fb[3] == 0xAA)
            print(f"  Footer OK: {footer_ok}")
        else:
            missing = expected_frame_end - len(bytes_arr)
            print(f"  Footer beyond data! Missing {missing} bytes")

        # results
        result_start = h_start + 4 + 4800
        if result_start + 7 < len(bytes_arr):
            rb = bytes_arr[result_start:result_start+8]
            print(f"  Result bytes: " + " ".join(f"0x{b:02X}" for b in rb))
            gap_pix = (rb[0] << 8) | rb[1]
            gap_mm = (rb[2] << 8) | rb[3]
            detect_row_lo = rb[4]
            status = rb[5]
            gap_left = rb[6]
            gap_right = rb[7]
            detect_row = ((status & 0x0C) << 6) | detect_row_lo
            print(f"  gap_pix={gap_pix} gap_mm_x10={gap_mm} detect_row={detect_row}")
            print(f"  status=0x{status:02X} (bin={status:08b})")
            print(f"    bit0 valid={(status & 1) != 0} bit1 stable={((status >> 1) & 1) != 0} bit4 v2={((status >> 4) & 1) != 0}")
            print(f"  gap_left={gap_left} gap_right={gap_right}")
        print()

# frame continuity
print("=== Frame Continuity ===")
for f_idx in range(len(headers) - 1):
    curr_end = headers[f_idx] + 4816
    next_start = headers[f_idx + 1]
    gap = next_start - curr_end
    print(f"  Frame {f_idx + 1} ends at {curr_end}, Frame {f_idx + 2} starts at {next_start}, gap={gap}")

# tail
print()
print("=== Tail check ===")
last_idx = len(bytes_arr) - 1
print(f"Last 16 bytes: " + " ".join(f"0x{b:02X}" for b in bytes_arr[max(0, last_idx-15):last_idx+1]))

# check for any spurious AA 55 AA 55 patterns inside image data
print()
print("=== Header-like patterns inside frame 1 image ===")
if len(headers) >= 1:
    h = headers[0]
    img_start = h + 4
    img_end = h + 4 + 4800
    for i in range(img_start, min(img_end, len(bytes_arr) - 3)):
        if bytes_arr[i] == 0xAA and bytes_arr[i+1] == 0x55 and bytes_arr[i+2] == 0xAA and bytes_arr[i+3] == 0x55:
            print(f"  Spurious header in image at index {i} (img offset {i - img_start})")
