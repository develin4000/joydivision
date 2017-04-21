sudo avrdude -c usbasp -P avrdoper -p atmega8 -U hfuse:w:0xc0:m -U lfuse:w:0x9f:m
sudo avrdude -c usbasp -P avrdoper -p atmega8 -U flash:w:main.hex:i