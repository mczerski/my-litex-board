/*
OUTPUT_FORMAT("elf32-littleriscv")
ENTRY(_start)
*/

MEMORY {
	srom : ORIGIN = 0x00000000, LENGTH = 0x00000400
}

SECTIONS
{
	.text :
	{
	} > srom

	.rodata :
	{
	} > srom

	.data :
	{
    } > srom

    /DISCARD/ :
    {
        *(.riscv.attributes)
        *(.comment)
    }

}
