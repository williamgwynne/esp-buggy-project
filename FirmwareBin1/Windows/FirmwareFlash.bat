esptool.exe --chip esp32 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0xe000 ./bin/boot_app0.bin 0x1000 ./bin/bootloader_qio_80m.bin 0x10000 ./bin/MainSrc.ino.bin 0x8000 ./bin/MainSrc.ino.partitions.bin
PAUSE
