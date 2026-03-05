@echo off
REM -------------------------------------------------
REM find_robot_ip.bat
REM Scan local network for robot using command port
REM -------------------------------------------------

setlocal

REM Robot command port
set PORT=10001
set SUBNET=192.168.1

echo Scanning network for robot...

for /L %%i in (1,1,254) do (
    REM Try TCP connection to each IP
    powershell -Command "$s=New-Object System.Net.Sockets.TcpClient; try {$s.Connect('%SUBNET%.%%i',%PORT%); Write-Host '%SUBNET%.%%i'; $s.Close()} catch {}"
)

endlocal