# CH340のCOMポートを自動検出 (COM7/COM8に自動追従)
$portDevice = Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match "CH340.*(COM\d+)" } | Select-Object -First 1
if ($portDevice -and ($portDevice.Name -match "(COM\d+)")) {
    $portName = $matches[1]
} else {
    $portName = "COM7"
}

Write-Output "=== AE86 POWERTRAIN TEST (Port: $portName) ==="

$port = New-Object System.IO.Ports.SerialPort $portName,115200
$port.DtrEnable = $false
$port.RtsEnable = $false
$port.Open()

Start-Sleep -Milliseconds 600

# 1. 前進 フルスロットル (約2.0秒)
Write-Output "`n[1/2] FORWARD DRIVE (100% POWER)"
for ($i = 0; $i -lt 25; $i++) {
    $port.WriteLine("c,255,0")
    Start-Sleep -Milliseconds 80
}
$port.WriteLine("c,0,0")
Write-Output "BRAKE / STOP"
Start-Sleep -Milliseconds 800

# 2. 後退 フルスロットル (約2.0秒)
Write-Output "`n[2/2] REVERSE DRIVE (100% POWER)"
for ($i = 0; $i -lt 25; $i++) {
    $port.WriteLine("c,-255,0")
    Start-Sleep -Milliseconds 80
}
$port.WriteLine("c,0,0")
Write-Output "BRAKE / STOP"
Start-Sleep -Milliseconds 800

# 停止
$port.WriteLine("s")
$port.Close()

Write-Output "`n=== TEST COMPLETE ==="
