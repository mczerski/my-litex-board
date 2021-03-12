#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.io import DDROutput
from litex.build.generic_platform import Subsignal, Pins, IOStandard

from litex_boards.platforms import de0nano

from litex.soc.cores.clock import CycloneIVPLL
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.gpio import GPIOIn

from litedram.modules import IS42S16160
from litedram.phy import GENSDRPHY, HalfRateGENSDRPHY

from liteeth.phy.rmii import LiteEthPHYRMII
from litex.soc.cores.spi import SPIMaster

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

class BaseSoC(SoCCore):
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
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident          = "LiteX SoC on DE0-Nano",
            ident_version  = True,
            **kwargs)

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
        self.submodules.leds = LedChaser(
            pads         = led_pads,
            sys_clk_freq = sys_clk_freq)
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

        # additional pin mappings
        #_spi_jp_ios = [
        #    ("spi", 0,
        #        Subsignal("cs_n", Pins("JP2:2 JP2:14")),
        #        Subsignal("miso", Pins("JP2:4")),
        #        Subsignal("mosi", Pins("JP2:6")),
        #        Subsignal("clk", Pins("JP2:8")),
        #        IOStandard("3.3-V LVTTL")
        #    ),
        #]
        #_interrupts_jp_ios = [
        #    ("interrupt", 0, Pins("JP2:10"), IOStandard("3.3-V LVTTL")),
        #    ("interrupt", 1, Pins("JP2:16"), IOStandard("3.3-V LVTTL")),
        #]
        #self.platform.add_extension(_spi_jp_ios)
        #self.platform.add_extension(_interrupts_jp_ios)

        # SPI
        #spi_pads = self.platform.request("spi")
        #self.submodules.spi = SPIMaster(spi_pads, 8, self.sys_clk_freq, 20e6)
        #self.spi.add_clk_divider()
        #self.add_csr("spi")
        #self.add_constant("SPI_NUM_CS", len(spi_pads.cs_n))

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

        # interrupts
        #interrupts_pads = self.platform.request_all("interrupt")
        #self.submodules.interrupts = GPIOIn(interrupts_pads, {0: 'fall', 1: 'fall'})
        #self.add_csr("interrupts")
        #self.irq.add("interrupts", use_loc_if_exists=True)
        #self.add_constant("INTERRUPTS_NGPIO", len(interrupts_pads))


        # Keys
        switches_pads = self.platform.request_all("key")
        self.submodules.switches = GPIOIn(switches_pads, {0: 'fall', 1: 'fall'})
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

    soc = BaseSoC(
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
