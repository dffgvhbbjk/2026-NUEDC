$content = Get-Content 'd:\fpgawork\God1.0\God1.0\shujv.txt' -Raw
$bytes = $content -split ', ' | Where-Object { $_ -match '0x[0-9A-Fa-f]+' }
Write-Host ('Total bytes: ' + $bytes.Count)
Write-Host ('First 10: ' + ($bytes[0..9] -join ' '))
Write-Host ('Last 10: ' + ($bytes[($bytes.Count-10)..($bytes.Count-1)] -join ' '))

# Find all occurrences of footer 55 AA 55 AA
$hexValues = $bytes | ForEach-Object { [Convert]::ToInt32($_, 16) }
Write-Host ('--- Searching for footer 55 AA 55 AA ---')
for ($i = 0; $i -lt $hexValues.Count - 3; $i++) {
    if ($hexValues[$i] -eq 0x55 -and $hexValues[$i+1] -eq 0xAA -and $hexValues[$i+2] -eq 0x55 -and $hexValues[$i+3] -eq 0xAA) {
        Write-Host ('Footer found at index: ' + $i)
    }
}

# Find all occurrences of header AA 55 AA 55
Write-Host ('--- Searching for header AA 55 AA 55 ---')
for ($i = 0; $i -lt $hexValues.Count - 3; $i++) {
    if ($hexValues[$i] -eq 0xAA -and $hexValues[$i+1] -eq 0x55 -and $hexValues[$i+2] -eq 0xAA -and $hexValues[$i+3] -eq 0x55) {
        Write-Host ('Header found at index: ' + $i)
    }
}

# Check measurement bytes (8 bytes before footer)
if ($hexValues.Count -ge 12) {
    $footerStart = $hexValues.Count - 4
    $measStart = $footerStart - 8
    Write-Host ('--- Measurement 8 bytes ---')
    $meas = $hexValues[$measStart..($footerStart-1)]
    Write-Host ('gap_pix_hi: ' + ('0x{0:X2}' -f $meas[0]))
    Write-Host ('gap_pix_lo: ' + ('0x{0:X2}' -f $meas[1]))
    Write-Host ('gap_mm_hi:  ' + ('0x{0:X2}' -f $meas[2]))
    Write-Host ('gap_mm_lo:  ' + ('0x{0:X2}' -f $meas[3]))
    Write-Host ('row_lo:     ' + ('0x{0:X2}' -f $meas[4]))
    Write-Host ('status:     ' + ('0x{0:X2}' -f $meas[5]))
    Write-Host ('gap_left:   ' + ('0x{0:X2}' -f $meas[6]))
    Write-Host ('gap_right:  ' + ('0x{0:X2}' -f $meas[7]))
}
