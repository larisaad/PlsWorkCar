all: lab0.hex

lab0.hex: lab0.elf
	avr-objcopy  -j .text -j .data -O ihex $^ $@
	avr-size lab0.elf

lab0.elf: test.c
	avr-gcc -mmcu=atmega324p -DF_CPU=16000000 -Os -Wall -o $@ $^

clean:
	rm -rf lab0.elf lab0.hex
