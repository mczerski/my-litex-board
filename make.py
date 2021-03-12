#!/usr/bin/env python3

import sys
import argparse
import os
import subprocess

from litex.soc.cores.cpu import VexRiscvSMP
from litex.soc.integration.soc_core import get_mem_data
from litex.soc.integration.builder import Builder

from soc_linux import SoCLinux

# Board definition----------------------------------------------------------------------------------

class Board:
    soc_kwargs = {"integrated_rom_size": 0x10000, "l2_size": 0}
    def __init__(self, soc_cls=None, soc_capabilities={}, bitstream_ext=""):
        self.soc_cls          = soc_cls
        self.soc_capabilities = soc_capabilities
        self.bitstream_ext    = bitstream_ext

    def load(self, filename):
        prog = self.platform.create_programmer()
        prog.load_bitstream(filename)

    def flash(self, filename):
        prog = self.platform.create_programmer()
        prog.flash(None, filename)

class De0Nano(Board):
    soc_kwargs = {
        "integrated_sram_size": 0x200,
        "integrated_rom_size": 0x400,
        "l2_size" : 2048,              # Use Wishbone and L2 for memory accesses.
    }
    def __init__(self):
        import my_de0nano
        Board.__init__(self, my_de0nano.BaseSoC, soc_capabilities={
            # Communication
            "serial",
        }, bitstream_ext=".sof")


#---------------------------------------------------------------------------------------------------
# Build
#---------------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--build",          action="store_true",      help="Build bitstream")
    parser.add_argument("--load",           action="store_true",      help="Load bitstream (to SRAM)")
    parser.add_argument("--flash",          action="store_true",      help="Flash bitstream/images (to SPI Flash)")
    VexRiscvSMP.args_fill(parser)
    args = parser.parse_args()

    # build bootrom
    subprocess.check_call("make", cwd="bootrom")

    board = De0Nano()
    board_name = "de0nano"
    soc_kwargs = Board.soc_kwargs
    rom_data = get_mem_data("bootrom/rom.bin", endianness="little", mem_size=soc_kwargs["integrated_rom_size"])
    soc_kwargs["integrated_rom_init"] = rom_data
    soc_kwargs.update(board.soc_kwargs)

    # CPU parameters ---------------------------------------------------------------------------
    # Do memory accesses through Wishbone and L2 cache when L2 size is configured.
    args.with_wishbone_memory = soc_kwargs["l2_size"] != 0
    VexRiscvSMP.args_read(args)

    # SoC creation -----------------------------------------------------------------------------
    soc = SoCLinux(board.soc_cls, **soc_kwargs)
    board.platform = soc.platform

    # Build ------------------------------------------------------------------------------------
    build_dir = os.path.join("build", board_name)
    builder   = Builder(soc,
        bios_options = ["TERM_MINI"],
        csr_json     = os.path.join(build_dir, "csr.json"),
        csr_csv      = os.path.join(build_dir, "csr.csv")
    )
    builder.build(run=args.build)

    # DTS --------------------------------------------------------------------------------------
    soc.generate_dts(board_name)
    soc.compile_dts(board_name)

    # Load FPGA bitstream ----------------------------------------------------------------------
    if args.load:
        board.load(filename=os.path.join(builder.gateware_dir, soc.build_name + board.bitstream_ext))

    # Flash bitstream/images (to SPI Flash) ----------------------------------------------------
    if args.flash:
        board.flash(filename=os.path.join(builder.gateware_dir, soc.build_name + board.bitstream_ext))

if __name__ == "__main__":
    main()
