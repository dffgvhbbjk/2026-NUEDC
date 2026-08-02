import re

with open(r"d:\fpgawork\God1.0\God1.0\shujv.txt", "r") as f:
    raw = f.read()

tokens = re.findall(r'0x([0-9A-Fa-f]{2})', raw)
bytes_arr = [int(t, 16) for t in tokens]

print(f"Total bytes: {len(bytes_arr)}")
print(f"Expected 2 frames = {2 * 4816} bytes")
print(f"Missing: {2 * 4816 - len(bytes_arr)} bytes")
print()

# Check frame 1 boundary (around index 4810-4820)
print("=== Around Frame 1 end / Frame 2 start (index 4800-4830) ===")
for i in range(4800, min(4830, len(bytes_arr))):
    print(f"  [{i}] = 0x{bytes_arr[i]:02X}")

print()
# Check frame 2 end (last 30 bytes)
print("=== Last 30 bytes ===")
start = len(bytes_arr) - 30
for i in range(start, len(bytes_arr)):
    print(f"  [{i}] = 0x{bytes_arr[i]:02X}")

print()
# Find exact footer of frame 1 (55 AA 55 AA)
print("=== Looking for footer 1 (55 AA 55 AA in first 4820 bytes) ===")
for i in range(4800, 4820):
    if bytes_arr[i] == 0x55 and bytes_arr[i+1] == 0xAA and bytes_arr[i+2] == 0x55 and bytes_arr[i+3] == 0xAA:
        print(f"  Footer 1 found at index: {i}")
        print(f"  Frame 1 total length: {i + 4} bytes (expected 4816, diff = {i + 4 - 4816})")
        break

print()
# Check what is at expected position 4812-4815
print("=== Expected frame 1 footer position (4812-4815) ===")
print(f"  [4812-4815]: " + " ".join(f"0x{bytes_arr[i]:02X}" for i in range(4812, 4816)))

# Check what is at expected position 4811-4814
print(f"  [4811-4814]: " + " ".join(f"0x{bytes_arr[i]:02X}" for i in range(4811, 4815)))

print()
# Calculate frame 1 actual length
print("=== Frame 1 actual layout ===")
print(f"  Header: 0-3 (4 bytes)")
print(f"  Image: 4 to ? (4800 bytes expected, ends at 4803)")
print(f"  Expected results position: 4804-4811 (8 bytes)")
print(f"  Expected footer position: 4812-4815 (4 bytes)")
print(f"  Expected frame end: 4816")
print()
print(f"  Actual footer found at: 4811")
print(f"  Actual frame 1 length: 4815 bytes (1 byte short!)")
print()
print(f"  Image bytes sent: 4799 (missing 1 byte)")
print(f"  Results bytes sent: 8 (correct)")
print(f"  Footer bytes sent: 4 (correct)")
print(f"  Total: 4 + 4799 + 8 + 4 = 4815 (off by 1)")
