# Check for Admin permissions
if (-not ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Write-Warning "Acest script necesita permisiuni de Administrator. Reporniti PowerShell ca Administrator!"
    exit 1
}

$usbRootName = "A4TECH USB Receiver"
$collectionNames = @{
    "Col01" = "A4TECH Bloody Gaming Wireless Keyboard"
    "Col02" = "A4TECH Consumer Media Control"
    "Col03" = "A4TECH Battery & Power Management"
    "Col04" = "A4TECH Wireless Configuration Interface"
    "Col05" = "A4TECH 1000Hz Ultra-Fast Diagnostic Interface"
}

# 1. Update USB Root entries
Write-Host "Configurare USB Root..." -ForegroundColor Cyan
Get-ChildItem "HKLM:\SYSTEM\CurrentControlSet\Enum\USB" -Recurse -ErrorAction SilentlyContinue | Where-Object { $_.PSChildName -match "VID_1B4F&PID_0001" } | ForEach-Object {
    Get-ChildItem $_.PSPath -ErrorAction SilentlyContinue | ForEach-Object {
        Set-ItemProperty -Path $_.PSPath -Name "FriendlyName" -Value $usbRootName -Force
        Set-ItemProperty -Path $_.PSPath -Name "DeviceDesc" -Value $usbRootName -Force
        Write-Host "  -> Setat: $($_.PSChildName) => $usbRootName" -ForegroundColor Green
    }
}

# 2. Update HID Collections
Write-Host "`nConfigurare Colectii HID..." -ForegroundColor Cyan
foreach ($col in $collectionNames.Keys) {
    $friendlyName = $collectionNames[$col]
    Get-ChildItem "HKLM:\SYSTEM\CurrentControlSet\Enum\HID" -ErrorAction SilentlyContinue | Where-Object { $_.PSChildName -match "VID_1B4F&PID_0001.*$col" } | ForEach-Object {
        Get-ChildItem $_.PSPath -ErrorAction SilentlyContinue | ForEach-Object {
            Set-ItemProperty -Path $_.PSPath -Name "FriendlyName" -Value $friendlyName -Force
            Set-ItemProperty -Path $_.PSPath -Name "DeviceDesc" -Value $friendlyName -Force
            Write-Host "  -> Setat: $($_.PSChildName) => $friendlyName" -ForegroundColor Green
        }
    }
}

Write-Host "`nToate colectiile A4TECH Receiver au fost configurate cu succes in Device Manager!" -ForegroundColor Green
