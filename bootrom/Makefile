all: rom.bin rom.d

rom.d: rom.elf
	riscv64-unknown-elf-objdump --disassemble-all $^ > $@

rom.bin: rom.elf
	riscv64-unknown-elf-objcopy -O binary $^ $@

rom.elf: rom.c
	riscv64-unknown-elf-gcc -march=rv32ima -mabi=ilp32 -fomit-frame-pointer -Wall -fno-builtin -nostdinc -nostartfiles -fno-stack-protector -O3 -Wl,--script=linker.ld $^ -o $@

clean:
	rm rom.d rom.elf rom.bin
