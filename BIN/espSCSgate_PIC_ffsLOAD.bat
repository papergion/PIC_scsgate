echo off
set DR=H:\Microchip Solutions\DOMOTIC\SCS\ESPSCSGATE\BIN
set ipaddress=192.168.2.34
set version=19_514

echo ipaddress %ipaddress%
echo __________________________________________SCS________________________________________
echo version: %version%
echo ipaddress: %ipaddress%
echo _____________________________________________________________________________________

if EXIST "%DR%\picscsgate_%version%.img" GOTO AVANTI
echo FILE NOT EXISTS
GOTO FINE

:AVANTI
echo - invia immagine spiffs da file hex di espscsgate

pause conferma
cd \Arduino\ESP8266\espota
espota.py -i %ipaddress% -f "%DR%\picscsgate_%version%.img" -s -r

:FINE
pause