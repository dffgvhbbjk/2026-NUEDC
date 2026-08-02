$content = Get-Content 'd:\fpgawork\God1.0\God1.0\shujv.txt' -Raw
$bytes = $content -split ', ' | Where-Object { $_ -match '0x[0-9A-Fa-f]+' }
$hexValues = $bytes | ForEach-Object { [Convert]::ToInt32($_, 16) }

Write-Host ('Total bytes: ' + $hexValues.Count)
Write-Host ''
Write-Host '=== Expected frame layout (4816 bytes) ==='
Write-Host 'Header:  0-3     (4 bytes)'
Write-Host 'Image:   4-4803  (4800 bytes)'
Write-Host 'Measure: 4804-4811 (8 bytes)'
Write-Host 'Footer:  4812-4815 (4 bytes)'
Write-Host ''

Write-Host '=== Bytes 4800-4820 (around frame 1 end / frame 2 start) ==='
for ($i = 4800; $i -lt 4821 -and $i -lt $hexValues.Count; $i++) {
    Write-Host ('  [{0,5}] = 0x{1:X2}' -f $i, $hexValues[$i])
}

Write-Host ''
Write-Host '=== Bytes 9615-9626 (end of data) ==='
for ($i = 9615; $i -lt $hexValues.Count; $i++) {
    Write-Host ('  [{0,5}] = 0x{1:X2}' -f $i, $hexValues[$i])
}

Write-Host ''
Write-Host '=== Analysis ==='
$expectedFrameLen = 4816
Write-Host ('Expected frame length: ' + $expectedFrameLen)
Write-Host ('Total data bytes: ' + $hexValues.Count)
Write-Host ('Number of complete frames expected: ' + [Math]::Floor($hexValues.Count / $expectedFrameLen))
Write-Host ('Remainder bytes: ' + ($hexValues.Count % $expectedFrameLen))

# Check if frame 1 is correct length
Write-Host ''
Write-Host '=== Frame 1 check ==='
$f1FooterStart = $expectedFrameLen - 4  # 4812
Write-Host ('Frame 1 footer should start at index: ' + $f1FooterStart)
Write-Host ('Bytes at footer position: ' + ('0x{0:X2} 0x{1:X2} 0x{2:X2} 0x{3:X2}' -f $hexValues[$f1FooterStart], $hexValues[$f1FooterStart+1], $hexValues[$f1FooterStart+2], $hexValues[$f1FooterStart+3]))

# Check if frame 2 starts at 4816
Write-Host ''
Write-Host '=== Frame 2 check ==='
$f2HeaderStart = $expectedFrameLen  # 4816
Write-Host ('Frame 2 header should start at index: ' + $f2HeaderStart)
if ($f2HeaderStart -lt $hexValues.Count) {
    Write-Host ('Bytes at header position: ' + ('0x{0:X2} 0x{1:X2} 0x{2:X2} 0x{3:X2}' -f $hexValues[$f2HeaderStart], $hexValues[$f2HeaderStart+1], $hexValues[$f2HeaderStart+2], $hexValues[$f2HeaderStart+3]))
}

# Check frame 2 measurement and footer
$f2MeasStart = $f2HeaderStart + 4 + 4800  # 4820 + 4800 = 9620
$f2FooterStart = $f2MeasStart + 8  # 9628
Write-Host ('Frame 2 measurement should start at index: ' + $f2MeasStart)
Write-Host ('Frame 2 footer should start at index: ' + $f2FooterStart)
Write-Host ('Data length: ' + $hexValues.Count)
if ($f2FooterStart + 3 -ge $hexValues.Count) {
    Write-Host 'FRAME 2 FOOTER IS BEYOND DATA - frame 2 is incomplete!'
    Write-Host ('Missing bytes: ' + ($f2FooterStart + 4 - $hexValues.Count))
}
