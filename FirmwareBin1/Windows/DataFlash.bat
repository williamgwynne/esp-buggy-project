esptool.exe --chip esp32 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x00290000 ./bin/MainSrc.spiffs.bin 
pause
