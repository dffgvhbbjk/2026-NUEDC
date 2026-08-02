# 分析 shujv.txt 的帧结构
$raw = Get-Content -Path "d:\fpgawork\God1.0\God1.0\shujv.txt" -Raw

# 使用更简单的字符串分割方式
$tokens = $raw -split '[,\s\]]+' | Where-Object { $_ -match '^0x[0-9A-Fa-f]{2}$' }
Write-Host ("Total tokens: " + $tokens.Count)

$bytes = New-Object int[] $tokens.Count
for ($i = 0; $i -lt $tokens.Count; $i++) {
    $bytes[$i] = [Convert]::ToInt32($tokens[$i].Substring(2), 16)
}

Write-Host ("Total bytes: " + $bytes.Count)
Write-Host ("Expected frame length: 4816 bytes (4+4800+8+4)")
Write-Host ("Expected frame count (if 4816-aligned): " + [Math]::Floor($bytes.Count / 4816))
Write-Host ("Remainder bytes: " + ($bytes.Count % 4816))
Write-Host ""

# 查找所有帧头位置 AA 55 AA 55
Write-Host "=== Header positions (AA 55 AA 55) ==="
$headers = @()
for ($i = 0; $i -lt $bytes.Count - 3; $i++) {
    if ($bytes[$i] -eq 0xAA -and $bytes[$i+1] -eq 0x55 -and $bytes[$i+2] -eq 0xAA -and $bytes[$i+3] -eq 0x55) {
        $headers += $i
    }
}
foreach ($h in $headers) { Write-Host ("  Header at index: " + $h) }
Write-Host ("Total headers found: " + $headers.Count)
Write-Host ""

# 查找所有帧尾位置 55 AA 55 AA
Write-Host "=== Footer positions (55 AA 55 AA) ==="
$footers = @()
for ($i = 0; $i -lt $bytes.Count - 3; $i++) {
    if ($bytes[$i] -eq 0x55 -and $bytes[$i+1] -eq 0xAA -and $bytes[$i+2] -eq 0x55 -and $bytes[$i+3] -eq 0xAA) {
        $footers += $i
    }
}
foreach ($f in $footers) { Write-Host ("  Footer at index: " + $f) }
Write-Host ("Total footers found: " + $footers.Count)
Write-Host ""

# 分析每帧
Write-Host "=== Frame Analysis ==="
if ($headers.Count -eq 0) {
    Write-Host "No headers found!"
    return
}

for ($f = 0; $f -lt $headers.Count; $f++) {
    $hStart = $headers[$f]
    $expectedFooterStart = $hStart + 4 + 4800 + 8  # 4812
    $expectedFrameEnd = $expectedFooterStart + 4    # 4816

    Write-Host ("--- Frame " + ($f + 1) + " ---")
    Write-Host ("  Header start: " + $hStart)
    Write-Host ("  Expected footer start: " + $expectedFooterStart)
    Write-Host ("  Expected frame end (exclusive): " + $expectedFrameEnd)

    # 检查 header
    $headerBytes = @($bytes[$hStart..($hStart+3)])
    Write-Host ("  Header bytes: " + (($headerBytes | ForEach-Object { '0x{0:X2}' -f $_ }) -join ' '))

    # 检查 footer
    if ($expectedFooterStart + 3 -lt $bytes.Count) {
        $footerBytes = @($bytes[$expectedFooterStart..($expectedFooterStart+3)])
        Write-Host ("  Footer bytes at expected pos: " + (($footerBytes | ForEach-Object { '0x{0:X2}' -f $_ }) -join ' '))
        $footerOk = ($footerBytes[0] -eq 0x55 -and $footerBytes[1] -eq 0xAA -and $footerBytes[2] -eq 0x55 -and $footerBytes[3] -eq 0xAA)
        Write-Host ("  Footer OK: " + $footerOk)
    } else {
        Write-Host ("  Footer beyond data! Missing " + ($expectedFrameEnd - $bytes.Count) + " bytes")
    }

    # 检查结果区 (8 字节, header+4800 开始)
    $resultStart = $hStart + 4 + 4800
    if ($resultStart + 7 -lt $bytes.Count) {
        $resultBytes = @($bytes[$resultStart..($resultStart+7)])
        Write-Host ("  Result bytes: " + (($resultBytes | ForEach-Object { '0x{0:X2}' -f $_ }) -join ' '))
        # 解析结果
        $gapPix = ($resultBytes[0] -shl 8) -bor $resultBytes[1]
        $gapMm = ($resultBytes[2] -shl 8) -bor $resultBytes[3]
        $detectRowLo = $resultBytes[4]
        $status = $resultBytes[5]
        $gapLeft = $resultBytes[6]
        $gapRight = $resultBytes[7]
        $detectRow = (($status -band 0x0C) -shl 6) -bor $detectRowLo
        Write-Host ("  gap_pix=" + $gapPix + " gap_mm_x10=" + $gapMm + " detect_row=" + $detectRow)
        $statusBin = [Convert]::ToString($status, 2).PadLeft(8, '0')
        Write-Host ("  status=0x" + ('{0:X2}' -f $status) + " (bin=" + $statusBin + ")")
        Write-Host ("    bit0 valid=" + (($status -band 1) -ne 0) + " bit1 stable=" + ((($status -shr 1) -band 1) -ne 0) + " bit4 v2=" + ((($status -shr 4) -band 1) -ne 0))
        Write-Host ("  gap_left=" + $gapLeft + " gap_right=" + $gapRight)
    }
    Write-Host ""
}

# 检查帧之间的连续性
Write-Host "=== Frame Continuity ==="
for ($f = 0; $f -lt $headers.Count - 1; $f++) {
    $currEnd = $headers[$f] + 4816
    $nextStart = $headers[$f + 1]
    $gap = $nextStart - $currEnd
    Write-Host ("  Frame " + ($f + 1) + " ends at " + $currEnd + ", Frame " + ($f + 2) + " starts at " + $nextStart + ", gap=" + $gap)
}

# 检查最后
Write-Host ""
Write-Host "=== Tail check ==="
$lastIdx = $bytes.Count - 1
Write-Host ("Last 16 bytes: " + (($bytes[($lastIdx-15)..$lastIdx] | ForEach-Object { '0x{0:X2}' -f $_ }) -join ' '))
