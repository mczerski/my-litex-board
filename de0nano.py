#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *

from litex.build.io import DDROutput
from litex.build.generic_platform import Subsignal, Pins, IOStandard, Misc

from litex.soc.cores.clock import CycloneIVPLL
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.cores.gpio import *
from litex.soc.cores.spi import SPIMaster
from litex.soc.cores.bitbang import I2CMaster

from litedram.modules import IS42S16160
from litedram.phy import GENSDRPHY, HalfRateGENSDRPHY

from liteeth.phy.rmii import LiteEthPHYRMII

from litex_boards.platforms import de0nano

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq, sdram_rate="1:1"):
        self.rst = Signal()
        self.clock_domains.cd_sys    = ClockDomain()
        if sdram_rate == "1:2":
            self.clock_domains.cd_sys2x    = ClockDomain()
            self.clock_domains.cd_sys2x_ps = ClockDomain(reset_less=True)
        else:
            self.clock_domains.cd_sys_ps = ClockDomain(reset_less=True)

        # # #

        # Clk / Rst
        clk50 = platform.request("clk50")

        # PLL
        self.submodules.pll = pll = CycloneIVPLL(speedgrade="-6")
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        if sdram_rate == "1:2":
            pll.create_clkout(self.cd_sys2x,    2*sys_clk_freq)
            pll.create_clkout(self.cd_sys2x_ps, 2*sys_clk_freq, phase=180)  # Idealy 90° but needs to be increased.
        else:
            pll.create_clkout(self.cd_sys_ps, sys_clk_freq, phase=90)

        # SDRAM clock
        sdram_clk = ClockSignal("sys2x_ps" if sdram_rate == "1:2" else "sys_ps")
        self.specials += DDROutput(1, 0, platform.request("sdram_clock"), sdram_clk)

# BaseSoC ------------------------------------------------------------------------------------------

class De0SoC(SoCCore):
    csr_map = {
        "ctrl": 0,
        "uart": 2,
        "timer0": 3,
    }
    interrupt_map = {
        "uart": 0,
        "timer0": 1,
    }
    mem_map = {
        "rom": 0x00000000,
        "main_ram": 0x40000000,
        "ethmac": 0xb0000000, # len: 0x2000
        "spiflash": 0xd0000000,
        "csr": 0xf0000000,
    }

    def __init__(self, sys_clk_freq=int(50e6), sdram_rate="1:1", **kwargs):
        platform = de0nano.Platform()
        platform.toolchain.additional_qsf_commands = [
            'set_global_assignment -name RESERVE_FLASH_NCE_AFTER_CONFIGURATION "USE AS REGULAR IO"',
            'set_global_assignment -name RESERVE_DATA0_AFTER_CONFIGURATION "USE AS REGULAR IO"',
            'set_global_assignment -name RESERVE_DATA1_AFTER_CONFIGURATION "USE AS REGULAR IO"',
            'set_global_assignment -name RESERVE_DCLK_AFTER_CONFIGURATION "USE AS REGULAR IO"',
        ]
        platform.toolchain.additional_sdc_commands = [
            'create_clock -name eth_rx_clk -period 20 [get_ports eth_clocks_ref_clk]',
            'create_clock -name eth_rx_clk_virt -period 20',
            'derive_clock_uncertainty',
            'set CLKAs_max 0.0',
            'set CLKAs_min 0.0',
            'set CLKAd_max 1.13',
            'set CLKAd_min 0.87',
            'set tCOa_max 16',
            'set tCOa_min 1',
            'set BDa_max 1.13',
            'set BDa_min 0.87',
            'set_input_delay -clock eth_rx_clk_virt -max [expr $CLKAs_max + $tCOa_max + $BDa_max - $CLKAd_min] [get_ports {eth_rx_data[*] eth_crs_dv}]',
            'set_input_delay -clock eth_rx_clk_virt -min [expr $CLKAs_min + $tCOa_min + $BDa_min - $CLKAd_max] [get_ports {eth_rx_data[*] eth_crs_dv}]',
        ]
        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(
            self,
            platform,
            sys_clk_freq,
            ident="LiteX SoC on DE0-Nano",
            ident_version=True,
            cpu_type="vexriscv_smp",
            cpu_variant="linux",
            max_sdram_size=0x40000000, # Limit mapped SDRAM to 1GB.
            **kwargs
        )

        # Add linker region for OpenSBI
        self.add_memory_region("opensbi", self.mem_map["main_ram"] + 0x00f00000, 0x80000, type="cached+linker")

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq, sdram_rate=sdram_rate)

        # SDR SDRAM --------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            sdrphy_cls = HalfRateGENSDRPHY if sdram_rate == "1:2" else GENSDRPHY
            self.submodules.sdrphy = sdrphy_cls(platform.request("sdram"), sys_clk_freq)
            self.add_sdram("sdram",
                phy                     = self.sdrphy,
                module                  = IS42S16160(sys_clk_freq, sdram_rate),
                origin                  = self.mem_map["main_ram"],
                size                    = kwargs.get("max_sdram_size", 0x40000000),
                l2_cache_size           = kwargs.get("l2_size", 8192),
                l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                l2_cache_reverse        = True
            )

        # Leds -------------------------------------------------------------------------------------
        led_pads = platform.request_all("user_led")
        self.submodules.leds = GPIOOut(led_pads)
        self.add_csr("leds")
        self.add_constant("LEDS_NGPIO", len(led_pads))

        # RMII Ethernet
        _ethernet_jp_ios = [
            ("eth_clocks", 0,
                Subsignal("ref_clk", Pins("JP2:34")),
                IOStandard("3.3-V LVTTL"),
            ),
            ("eth", 0,
                #Subsignal("rst_n",   Pins("")),
                Subsignal("rx_data", Pins("JP2:36 JP2:35")),
                Subsignal("crs_dv",  Pins("JP2:33")),
                Subsignal("tx_en",   Pins("JP2:38")),
                Subsignal("tx_data", Pins("JP2:37 JP2:40")),
                Subsignal("mdc",     Pins("JP2:31")),
                Subsignal("mdio",    Pins("JP2:32")),
                #Subsignal("rx_er",   Pins("")),
                #Subsignal("int_n",   Pins("")),
                IOStandard("3.3-V LVTTL")
            ),
        ]
        self.platform.add_extension(_ethernet_jp_ios)
        self.submodules.ethphy = LiteEthPHYRMII(
            clock_pads = self.platform.request("eth_clocks"),
            pads       = self.platform.request("eth"),
            refclk_cd  = None,
        )
        self.add_csr("ethphy")
        self.add_ethernet(phy=self.ethphy, nrxslots=4, ntxslots=2)

        # SD Card
        _sdcard_jp_ios = [
            ("sdcard", 0,
                Subsignal("data", Pins("JP2:14 JP2:24 JP2:22 JP2:20"), Misc("WEAK_PULL_UP_RESISTOR ON")),
                Subsignal("cmd", Pins("JP2:16"), Misc("WEAK_PULL_UP_RESISTOR ON")),
                Subsignal("clk", Pins("JP2:18")),
                Subsignal("cd", Pins("JP2:26")),
                Misc("FAST_OUTPUT_REGISTER ON"),
                IOStandard("3.3-V LVTTL"),
            ),
        ]
        self.platform.add_extension(_sdcard_jp_ios)
        self.add_sdcard()

        # EPCS
        _spi_flash_ios = [
            ("spiflash", 0,
                Subsignal("miso", Pins("H2")),
                Subsignal("clk",  Pins("H1")),
                Subsignal("cs_n",  Pins("D2")),
                Subsignal("mosi",  Pins("C1")),
                IOStandard("3.3-V LVTTL")
            ),
        ]
        self.platform.add_extension(_spi_flash_ios)
        self.add_spi_flash(mode="1x", dummy_cycles=8)

        # I2C ADXL345
        _i2c_ios = [
            ("i2c_onboard", 0,
                Subsignal("scl", Pins("F2")),
                Subsignal("sda", Pins("F1")),
                IOStandard("3.3-V LVTTL")
            ),
        ]
        self.platform.add_extension(_i2c_ios)
        self.submodules.i2c0 = I2CMaster(self.platform.request("i2c_onboard", 0))
        self.add_csr("i2c0")
        adxl_pads = self.platform.request("acc")
        self.comb += adxl_pads.cs_n.eq(1)

        # interrupts
        #_interrupts_jp_ios = [
        #    ("interrupt", 0, Pins("JP2:1"),  IOStandard("3.3-V LVTTL")),
        #    ("interrupt", 1, Pins("JP2:3"),  IOStandard("3.3-V LVTTL")),
        #]
        #self.platform.add_extension(_interrupts_jp_ios)
        #interrupts_pads = self.platform.request_all("interrupt")
        interrupts_pads = adxl_pads.int
        self.submodules.interrupts = GPIOIn(interrupts_pads, with_irq=True)
        self.add_csr("interrupts")
        self.irq.add("interrupts", use_loc_if_exists=True)
        self.add_constant("INTERRUPTS_NGPIO", len(interrupts_pads))

        # Keys
        switches_pads = self.platform.request_all("key")
        self.submodules.switches = GPIOIn(pads=switches_pads, with_irq=True)
        self.add_csr("switches")
        self.irq.add("switches", use_loc_if_exists=True)
        self.add_constant("SWITCHES_NGPIO", len(switches_pads))

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on DE0-Nano")
    parser.add_argument("--build",        action="store_true", help="Build bitstream")
    parser.add_argument("--load",         action="store_true", help="Load bitstream")
    parser.add_argument("--sys-clk-freq", default=50e6,        help="System clock frequency (default: 50MHz)")
    parser.add_argument("--sdram-rate",   default="1:1",       help="SDRAM Rate: 1:1 Full Rate (default), 1:2 Half Rate")
    builder_args(parser)
    soc_sdram_args(parser)
    args = parser.parse_args()

    soc = De0SoC(
        sys_clk_freq = int(float(args.sys_clk_freq)),
        sdram_rate   = args.sdram_rate,
        **soc_sdram_argdict(args)
    )
    builder = Builder(soc, **builder_argdict(args))
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".sof"))

if __name__ == "__main__":
    main()
