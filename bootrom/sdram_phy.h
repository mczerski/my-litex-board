#ifndef __SDRAM_PHY_H
#define __SDRAM_PHY_H

#define DFII_CONTROL_SEL        0x01
#define DFII_CONTROL_CKE        0x02
#define DFII_CONTROL_ODT        0x04
#define DFII_CONTROL_RESET_N    0x08

#define DFII_COMMAND_CS         0x01
#define DFII_COMMAND_WE         0x02
#define DFII_COMMAND_CAS        0x04
#define DFII_COMMAND_RAS        0x08
#define DFII_COMMAND_WRDATA     0x10
#define DFII_COMMAND_RDDATA     0x20

#define SDRAM_PHY_GENSDRPHY
#define SDRAM_PHY_XDR 1
#define SDRAM_PHY_DATABITS 16
#define SDRAM_PHY_PHASES 1
#define SDRAM_PHY_CL 2
#define SDRAM_PHY_CWL 2
#define SDRAM_PHY_RDPHASE 0
#define SDRAM_PHY_WRPHASE 0

#define CSR_BASE 0xf0000000L

/* CSR subregisters (a.k.a. "simple CSRs") are embedded inside unsigned
 * aligned locations: */
#define MMPTR(a) (*((volatile unsigned *)(a)))

static inline void csr_write_simple(unsigned long v, unsigned long a) 
{ 
    MMPTR(a) = v; 
}

static inline unsigned long csr_read_simple(unsigned long a)
{
    return MMPTR(a);
}

static inline void sdram_dfii_pi0_command_write(unsigned v) {
	csr_write_simple(v, CSR_BASE + 0x3004L);
}

static inline void sdram_dfii_pi0_command_issue_write(unsigned v) {
	csr_write_simple(v, CSR_BASE + 0x3008L);
}

static inline void sdram_dfii_pi0_address_write(unsigned v) {
	csr_write_simple(v, CSR_BASE + 0x300cL);
}

static inline void sdram_dfii_pi0_baddress_write(unsigned v) {
	csr_write_simple(v, CSR_BASE + 0x3010L);
}

static inline void sdram_dfii_control_write(unsigned v) {
	csr_write_simple(v, CSR_BASE + 0x3000L);
}

static void cdelay(int i)
{
    while(i > 0) {
        __asm__ volatile("nop");
        i--;
    }
}

__attribute__((unused)) static void command_p0(int cmd)
{
    sdram_dfii_pi0_command_write(cmd);
    sdram_dfii_pi0_command_issue_write(1);
}

static inline unsigned sdram_dfii_control_read(void) {
    return csr_read_simple(CSR_BASE + 0x3000L);
}


#define DFII_CONTROL_SOFTWARE (DFII_CONTROL_CKE|DFII_CONTROL_ODT|DFII_CONTROL_RESET_N)
#define DFII_CONTROL_HARDWARE (DFII_CONTROL_SEL)

static inline void sdram_software_control_on(void)
{
    unsigned int previous;
    previous = sdram_dfii_control_read();
    /* Switch DFII to software control */
    if (previous != DFII_CONTROL_SOFTWARE) {
        sdram_dfii_control_write(DFII_CONTROL_SOFTWARE);
    }
}

static inline void sdram_software_control_off(void)
{
    unsigned int previous;
    previous = sdram_dfii_control_read();
    /* Switch DFII to hardware control */
    if (previous != DFII_CONTROL_HARDWARE) {
        sdram_dfii_control_write(DFII_CONTROL_HARDWARE);
    }
}
static inline void init_sequence(void)
{
    sdram_software_control_on();

	/* Bring CKE high */
	sdram_dfii_pi0_address_write(0x0);
	sdram_dfii_pi0_baddress_write(0);
	sdram_dfii_control_write(DFII_CONTROL_CKE|DFII_CONTROL_ODT|DFII_CONTROL_RESET_N);
	cdelay(20000);

	/* Precharge All */
	sdram_dfii_pi0_address_write(0x400);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);

	/* Load Mode Register / Reset DLL, CL=2, BL=1 */
	sdram_dfii_pi0_address_write(0x120);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
	cdelay(200);

	/* Precharge All */
	sdram_dfii_pi0_address_write(0x400);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_WE|DFII_COMMAND_CS);

	/* Auto Refresh */
	sdram_dfii_pi0_address_write(0x0);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_CS);
	cdelay(4);

	/* Auto Refresh */
	sdram_dfii_pi0_address_write(0x0);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_CS);
	cdelay(4);

	/* Load Mode Register / CL=2, BL=1 */
	sdram_dfii_pi0_address_write(0x20);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
	cdelay(200);

    sdram_software_control_off();
}
#endif
