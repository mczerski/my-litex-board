#!/usr/bin/env python3

import argparse
import os
import subprocess
import json

from litex.soc.cores.cpu import VexRiscvSMP
from litex.soc.integration.soc_core import get_mem_data
from litex.soc.integration.builder import Builder
from litex.tools import litex_json2dts

from de0nano import De0SoC

#---------------------------------------------------------------------------------------------------
# Build
#---------------------------------------------------------------------------------------------------

def generate_dts(board_name):
    json_src = os.path.join("build", board_name, "csr.json")
    dts = os.path.join("build", board_name, "{}.dts".format(board_name))

    with open(json_src) as json_file, open(dts, "w") as dts_file:
        dts_content = litex_json2dts.generate_dts(json.load(json_file), polling=False)
        dts_file.write(dts_content)

def compile_dts(board_name):
    dts = os.path.join("build", board_name, "{}.dts".format(board_name))
    dtb = os.path.join("images", "rv32.dtb")
    subprocess.check_call(
        "dtc -O dtb -o {} {}".format(dtb, dts), shell=True)

def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--build", action="store_true", help="Build bitstream")
    parser.add_argument("--load", action="store_true", help="Load bitstream (to SRAM)")
    VexRiscvSMP.args_fill(parser)
    args = parser.parse_args()

    # build bootrom
    subprocess.check_call("make", cwd="bootrom")

    # CPU parameters ---------------------------------------------------------------------------
    # Do memory accesses through Wishbone and L2 cache when L2 size is configured.
    args.with_wishbone_memory = True
    VexRiscvSMP.args_read(args)

    # SoC creation -----------------------------------------------------------------------------
    soc_kwargs = {
        "integrated_sram_size": 0,
        "integrated_rom_size": 0x400,
        "l2_size" : 2048, # Use Wishbone and L2 for memory accesses.
        "uart_baudrate": 1e6,
        "uart_fifo_depth": 1024,
    }
    rom_data = get_mem_data("bootrom/rom.bin", endianness="little", mem_size=soc_kwargs["integrated_rom_size"])
    soc_kwargs["integrated_rom_init"] = rom_data
    soc = De0SoC(**soc_kwargs)
    board_name = "de0nano"

    # Build ------------------------------------------------------------------------------------
    build_dir = os.path.join("build", board_name)
    builder   = Builder(
        soc,
        csr_json=os.path.join(build_dir, "csr.json"),
        csr_csv=os.path.join(build_dir, "csr.csv"),
        compile_software=False
    )
    builder.build(run=args.build)

    # DTS --------------------------------------------------------------------------------------
    generate_dts(board_name)
    compile_dts(board_name)

    # Load FPGA bitstream ----------------------------------------------------------------------
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".sof"))

if __name__ == "__main__":
    main()
