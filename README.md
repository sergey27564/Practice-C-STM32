# Practice C STM32
Transmitting messages under VCOM port with headers and checksum

First 2 bytes (0xaa 0x55) is header
Third byte is length of message
4 and 5 bytes is control bytes (if you want Set/reset D0 or D4 port - 4 and 5 bytes, if you want to recive byte from 0 to 100 - only 4)
5/6 byte is CRC-8 checkbyte
