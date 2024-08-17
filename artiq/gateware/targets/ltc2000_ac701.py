from migen import *
#from migen.fhdl import verilog
from migen.build.platforms import ac701
from migen.build.generic_platform import *
from misoc.cores.duc import PhasedAccu
from misoc.cores.duc import CosSinGen
from misoc.targets.acltc import BaseSoC
#from artiq.gateware.rtio.phy import ttl_serdes_7series
from artiq.gateware.drtio.rx_synchronizer import XilinxRXSynchronizer
from artiq.gateware.drtio import *
from artiq.gateware.ltc2000phy import Ltc2000phy
from artiq.gateware.amp import AMPSoC

# ~ class StandaloneBase(MiniSoC):

    # ~ def __init__(self, gateware_identifier_str=None, **kwargs):
        # ~ cpu_bus_width = 64
        # ~ MiniSoC.__init__(self,
                         # ~ cpu_type="vexriscv",
                         # ~ hw_rev=hw_rev,
                         # ~ cpu_bus_width=cpu_bus_width,
                         # ~ sdram_controller_type="minicon",
                         # ~ l2_size=128*1024,
                         # ~ integrated_sram_size=8192,
                         # ~ ethmac_nrxslots=4,
                         # ~ ethmac_ntxslots=4,
                         # ~ clk_freq=kwargs.get("rtio_frequency", 125.0e6),
                         # ~ rtio_sys_merge=True,
                         # ~ **kwargs)
        # ~ add_identifier(self, gateware_identifier_str=gateware_identifier_str)

class SatelliteBase(BaseSoC, AMPSoC):
    mem_map = {
        "rtio":          0x20000000,
        "drtioaux":      0x50000000,
        "mailbox":       0x70000000
    }
    mem_map.update(BaseSoC.mem_map)

    def __init__(self, rtio_clk_freq=100e6, *, gateware_identifier_str=None, hw_rev="v2.0", **kwargs):
        cpu_bus_width = 64

        BaseSoC.__init__(self,
                 cpu_type="vexriscv",
                 hw_rev=hw_rev,
                 cpu_bus_width=cpu_bus_width,
                 sdram_controller_type="minicon",
                 l2_size=128*1024,
                 clk_freq=rtio_clk_freq,
                 rtio_sys_merge=True,
                 **kwargs)
        AMPSoC.__init__(self)
        add_identifier(self, gateware_identifier_str=gateware_identifier_str)

        platform = self.platform

        platform.add_period_constraint(self.dac_pads.dcko_p, 1.66)
        ltcclk600_se = Signal()
        self.specials += Instance("IBUFDS",
            i_I=self.dac_pads.dcko_p,
            i_IB=self.dac_pads.dcko_n,
            o_O=ltcclk600_se
        )       

        pll_clk100 = Signal()
        pll_clk200 = Signal()
        pll_clk400 = Signal()
        pll_fb = Signal()
        self.pll_locked = Signal()
        self.specials += [
              Instance("PLLE2_BASE",
                  p_CLKIN1_PERIOD=1.67,
                  i_CLKIN1=ltcclk600_se,

                  i_CLKFBIN=pll_fb,
                  o_CLKFBOUT=pll_fb,
                  o_LOCKED=self.pll_locked,

                  # VCO @ 1.2GHz
                  p_CLKFBOUT_MULT=2, p_DIVCLK_DIVIDE=1,

                  # 200MHz for IDELAYCTRL
                  p_CLKOUT0_DIVIDE=6, p_CLKOUT0_PHASE=0.0, o_CLKOUT0=pll_clk200,
                  # 100MHz cd_sys
                  p_CLKOUT1_DIVIDE=12, p_CLKOUT1_PHASE=0.0, o_CLKOUT1=pll_clk100,
                  # 400MHz cd_sys4x
                  p_CLKOUT2_DIVIDE=2, p_CLKOUT2_PHASE=0.0, o_CLKOUT2=pll_clk400
              ),
              Instance("BUFG", i_I=pll_clk100, o_O=self.cd_sys.clk),
              Instance("BUFG", i_I=pll_clk400, o_O=self.cd_sys4x.clk),
              Instance("BUFG", i_I=pll_clk200, o_O=self.cd_clk200.clk)
        ]

        cdr_clk = Signal()
        cdr_clk.eq(self.cd_sys.clk)
        qpll_drtio_settings = QPLLSettings(
            refclksel=0b001,
            fbdiv=4,
            fbdiv_45=5,
            refclk_div=1)
        qpll = QPLL(cdr_clk, qpll_drtio_settings)
        self.submodules += qpll

        drtio_data_pads = [platform.request("sfp_rx", 0), platform.request("sfp_tx", 0)]
        sfp_tx_disable_n = platform.request('sfp_tx_disable_n',0)
        sfp_tx_disable_n.eq(1)
        self.submodules.gt_drtio = gtp_7series.GTP(
            qpll_channel=qpll.channels[0],
            data_pads=drtio_data_pads,
            sys_clk_freq=self.clk_freq,
            rtio_clk_freq=rtio_clk_freq)
        self.csr_devices.append("gt_drtio")

        self.submodules.rtio_tsc = rtio.TSC(glbl_fine_ts_width=3)

        drtioaux_csr_group = []
        drtioaux_memory_group = []
        drtiorep_csr_group = []
        self.drtio_cri = []
        for i, channel in enumerate(self.gt_drtio.channels):
            coreaux_name = "drtioaux" + str(i)
            memory_name = "drtioaux" + str(i) + "_mem"
            drtioaux_csr_group.append(coreaux_name)
            drtioaux_memory_group.append(memory_name)

            cdr = ClockDomainsRenamer({"rtio_rx": "rtio_rx" + str(i)})

            if i == 0:
                self.submodules.rx_synchronizer = cdr(XilinxRXSynchronizer())
                core = cdr(DRTIOSatellite(
                    self.rtio_tsc, channel,
                    self.rx_synchronizer))
                self.submodules.drtiosat = core
                self.csr_devices.append("drtiosat")
            else:
                corerep_name = "drtiorep" + str(i-1)
                drtiorep_csr_group.append(corerep_name)

                core = cdr(DRTIORepeater(
                    self.rtio_tsc, channel))
                setattr(self.submodules, corerep_name, core)
                self.drtio_cri.append(core.cri)
                self.csr_devices.append(corerep_name)

            coreaux = cdr(DRTIOAuxController(core.link_layer, self.cpu_dw))
            setattr(self.submodules, coreaux_name, coreaux)
            self.csr_devices.append(coreaux_name)

            memory_address = self.mem_map["drtioaux"] + 0x800*i
            self.add_wb_slave(memory_address, 0x800,
                              coreaux.bus)
            self.add_memory_region(memory_name, memory_address | self.shadow_base, 0x800)
        self.config["HAS_DRTIO"] = None
        self.config["HAS_DRTIO_ROUTING"] = None
        self.config["DRTIO_ROLE"] = "satellite"
        self.add_csr_group("drtioaux", drtioaux_csr_group)
        self.add_memory_group("drtioaux_mem", drtioaux_memory_group)
        self.add_csr_group("drtiorep", drtiorep_csr_group)

        self.config["I2C_BUS_COUNT"] = 0

        rtio_clk_period = 1e9/rtio_clk_freq
        self.config["RTIO_FREQUENCY"] = str(rtio_clk_freq/1e6)

        self.submodules.siphaser = SiPhaser7Series(
            si5324_clkin=platform.request("cdr_clk") if platform.hw_rev == "v2.0"
                else platform.request("si5324_clkin"),
            rx_synchronizer=self.rx_synchronizer,
            ref_clk=self.crg.clk125_div2, ref_div2=True,
            rtio_clk_freq=rtio_clk_freq)
        platform.add_false_path_constraints(
            self.crg.cd_sys.clk, self.siphaser.mmcm_freerun_output)
        self.csr_devices.append("siphaser")
        self.config["HAS_SI5324"] = None
        self.config["SI5324_SOFT_RESET"] = None

        gtp = self.gt_drtio.gtps[0]
        txout_buf = Signal()
        self.specials += Instance("BUFG", i_I=gtp.txoutclk, o_O=txout_buf)
        self.crg.configure(txout_buf, clk_sw=self.gt_drtio.stable_clkin.storage, ext_async_rst=self.crg.clk_sw_fsm.o_clk_sw & ~gtp.tx_init.done)
        self.specials += MultiReg(self.crg.clk_sw_fsm.o_clk_sw & self.crg.mmcm_locked, self.gt_drtio.clk_path_ready, odomain="bootstrap")

        platform.add_period_constraint(gtp.txoutclk, rtio_clk_period)
        platform.add_period_constraint(gtp.rxoutclk, rtio_clk_period)
        platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            gtp.txoutclk, gtp.rxoutclk)
        for gtp in self.gt_drtio.gtps[1:]:
            platform.add_period_constraint(gtp.rxoutclk, rtio_clk_period)
            platform.add_false_path_constraints(
                self.crg.cd_sys.clk, gtp.rxoutclk)

        fix_serdes_timing_path(platform)

    def add_rtio(self, rtio_channels, sed_lanes=8):
        # Only add MonInj core if there is anything to monitor
        if any([len(c.probes) for c in rtio_channels]):
            self.submodules.rtio_moninj = rtio.MonInj(rtio_channels)
            self.csr_devices.append("rtio_moninj")

        # satellite (master-controlled) RTIO
        self.submodules.local_io = SyncRTIO(self.rtio_tsc, rtio_channels, lane_count=sed_lanes)
        self.comb += self.drtiosat.async_errors.eq(self.local_io.async_errors)

        # subkernel RTIO
        self.submodules.rtio = rtio.KernelInitiator(self.rtio_tsc)
        self.register_kernel_cpu_csrdevice("rtio")

        self.submodules.rtio_dma = rtio.DMA(self.get_native_sdram_if(), self.cpu_dw)
        self.csr_devices.append("rtio_dma")
        self.submodules.cri_con = rtio.CRIInterconnectShared(
            [self.drtiosat.cri, self.rtio_dma.cri, self.rtio.cri],
            [self.local_io.cri] + self.drtio_cri,
            enable_routing=True)
        self.csr_devices.append("cri_con")
        self.submodules.routing_table = rtio.RoutingTableAccess(self.cri_con)
        self.csr_devices.append("routing_table")

        self.submodules.rtio_analyzer = rtio.Analyzer(self.rtio_tsc, self.local_io.cri,
                                                self.get_native_sdram_if(), cpu_dw=self.cpu_dw)
        self.csr_devices.append("rtio_analyzer")

class GenericSatellite(SatelliteBase):
    def __init__(self, description, hw_rev=None, **kwargs):
        if hw_rev is None:
            hw_rev = description["hw_rev"]
        self.class_name_override = description["variant"]
        SatelliteBase.__init__(self,
                               hw_rev=hw_rev,
                               rtio_clk_freq=description["rtio_frequency"],
                               **kwargs)
        if hw_rev == "v1.0":
            # EEM clock fan-out from Si5324, not MMCX
            self.comb += self.platform.request("clk_sel").eq(1)

        has_grabber = any(peripheral["type"] == "grabber" for peripheral in description["peripherals"])
        if has_grabber:
            self.grabber_csr_group = []

        self.rtio_channels = []
        eem_7series.add_peripherals(self, description["peripherals"])
        if hw_rev in ("v1.1", "v2.0"):
            for i in range(3):
                print("USER LED at RTIO channel 0x{:06x}".format(len(self.rtio_channels)))
                phy = ttl_simple.Output(self.platform.request("user_led", i))
                self.submodules += phy
                self.rtio_channels.append(rtio.Channel.from_phy(phy))

        self.config["HAS_RTIO_LOG"] = None
        self.config["RTIO_LOG_CHANNEL"] = len(self.rtio_channels)
        self.rtio_channels.append(rtio.LogChannel())

        self.add_rtio(self.rtio_channels, sed_lanes=description["sed_lanes"])
        if has_grabber:
            self.config["HAS_GRABBER"] = None
            self.add_csr_group("grabber", self.grabber_csr_group)
            for grabber in self.grabber_csr_group:
                self.platform.add_false_path_constraints(
                    self.gt_drtio.gtps[0].txoutclk, getattr(self, grabber).deserializer.cd_cl.clk)

class PolyphaseDDS(Module):
    """Composite DDS with sub-DDSs synthesizing\n",
       individual phases to increase fmax.\n",
    """
    def __init__(self, n, fwidth, pwidth, z=18, x=15, zl=9, xd=4, backoff=None, share_lut=None):
        self.ftw  = Signal(fwidth)   # input frequency tuning word
        self.ptw  = Signal(pwidth)   # input phase tuning word
        self.clr  = Signal()         # input clear signal
        self.dout = Signal((x+1)*n)  # output data

        ###

        paccu = PhasedAccu(n, fwidth, pwidth)
        self.comb += paccu.clr.eq(self.clr)
        self.comb += paccu.f.eq(self.ftw)
        self.comb += paccu.p.eq(self.ptw)
        self.submodules.paccu = paccu
        ddss = [CosSinGen() for i in range(n)]
        for idx, dds in enumerate(ddss):
            self.submodules += dds
            self.comb += dds.z.eq(paccu.z[idx])
            self.comb += self.dout[idx*16:(idx+1)*16].eq(dds.y)


class DACTest(Module):

    def __init__(self):

        self.platform = ac701.Platform()
        platform = self.platform

        dac_pads = []
        dac_pads.append(("ltc2000", 0,
                Subsignal("clk_p", Pins("HPC:LA07_P"), IOStandard("LVDS_25")),
                Subsignal("clk_n", Pins("HPC:LA07_N"), IOStandard("LVDS_25")),
                Subsignal("cs", Pins("HPC:HA21_N"), IOStandard("LVCMOS25")),
                Subsignal("sck", Pins("HPC:HA17_CC_P"), IOStandard("LVCMOS25")),
                Subsignal("sdi", Pins("HPC:HA17_CC_N"), IOStandard("LVCMOS25")),
                Subsignal("sdo", Pins("HPC:HA21_P"), IOStandard("LVCMOS25")),
                Subsignal("dcko_p", Pins("HPC:LA01_CC_P"), IOStandard("LVDS_25")),
                Subsignal("dcko_n", Pins("HPC:LA01_CC_N"), IOStandard("LVDS_25")),
                Subsignal("data_p", Pins(
                    "HPC:LA15_P HPC:LA16_P HPC:LA14_P",
                    "HPC:LA13_P HPC:LA11_P HPC:LA12_P",
                    "HPC:LA09_P HPC:LA10_P HPC:LA08_P",
                    "HPC:LA05_P HPC:LA04_P HPC:LA06_P",
                    "HPC:LA03_P HPC:LA02_P HPC:LA00_CC_P",
                    "HPC:CLK0_M2C_P"),
                    IOStandard("LVDS_25")),
                Subsignal("data_n", Pins(
                    "HPC:LA15_N HPC:LA16_N HPC:LA14_N",
                    "HPC:LA13_N HPC:LA11_N HPC:LA12_N",
                    "HPC:LA09_N HPC:LA10_N HPC:LA08_N",
                    "HPC:LA05_N HPC:LA04_N HPC:LA06_N",
                    "HPC:LA03_N HPC:LA02_N HPC:LA00_CC_N",
                    "HPC:CLK0_M2C_N"),
                    IOStandard("LVDS_25")),
                Subsignal("datb_p", Pins(
                    "HPC:LA32_P HPC:LA33_P HPC:LA30_P",
                    "HPC:LA31_P HPC:LA28_P HPC:LA29_P",
                    "HPC:LA24_P HPC:LA25_P HPC:LA26_P",
                    "HPC:LA27_P HPC:LA21_P HPC:LA22_P",
                    "HPC:LA23_P HPC:LA19_P HPC:LA20_P",
                    "HPC:LA17_CC_P"),
                    IOStandard("LVDS_25")),
                Subsignal("datb_n", Pins(
                    "HPC:LA32_N HPC:LA33_N HPC:LA30_N",
                    "HPC:LA31_N HPC:LA28_N HPC:LA29_N",
                    "HPC:LA24_N HPC:LA25_N HPC:LA26_N",
                    "HPC:LA27_N HPC:LA21_N HPC:LA22_N",
                    "HPC:LA23_N HPC:LA19_N HPC:LA20_N",
                    "HPC:LA17_CC_N"),
                    IOStandard("LVDS_25")))
        )

        platform.add_extension(dac_pads)

        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_sys4x = ClockDomain(reset_less=True)
        self.clock_domains.cd_clk200 = ClockDomain()
        self.dac_pads = platform.request("ltc2000")

        platform.add_period_constraint(self.dac_pads.dcko_p, 1.66)
        ltcclk600_se = Signal()
        self.specials += Instance("IBUFDS",
            i_I=self.dac_pads.dcko_p,
            i_IB=self.dac_pads.dcko_n,
            o_O=ltcclk600_se
        )
        pll_clk200 = Signal()
        pll_clk100 = Signal()
        pll_clk400 = Signal()
        pll_fb = Signal()
        self.pll_locked = Signal()
        self.specials += [
              Instance("PLLE2_BASE",
                  p_CLKIN1_PERIOD=1.67,
                  i_CLKIN1=ltcclk600_se,

                  i_CLKFBIN=pll_fb,
                  o_CLKFBOUT=pll_fb,
                  o_LOCKED=self.pll_locked,

                  # VCO @ 1.2GHz
                  p_CLKFBOUT_MULT=2, p_DIVCLK_DIVIDE=1,

                  # 200MHz for IDELAYCTRL
                  p_CLKOUT0_DIVIDE=6, p_CLKOUT0_PHASE=0.0, o_CLKOUT0=pll_clk200,
                  # 100MHz cd_sys
                  p_CLKOUT1_DIVIDE=12, p_CLKOUT1_PHASE=0.0, o_CLKOUT1=pll_clk100,
                  # 400MHz cd_sys4x
                  p_CLKOUT2_DIVIDE=2, p_CLKOUT2_PHASE=0.0, o_CLKOUT2=pll_clk400
              ),
              Instance("BUFG", i_I=pll_clk100, o_O=self.cd_sys.clk),
              Instance("BUFG", i_I=pll_clk400, o_O=self.cd_sys4x.clk),
              Instance("BUFG", i_I=pll_clk200, o_O=self.cd_clk200.clk)
        ]

        self.submodules.ltc2000 = Ltc2000phy(self.dac_pads)
        self.submodules.dds = PolyphaseDDS(16,32,18)
        self.comb += self.dds.ftw.eq(100000000)        # input frequency tuning word. 2400/2^32*FTW MHz
        self.comb += self.dds.ptw.eq(0)               # input phase tuning word
        self.comb += self.dds.clr.eq(0)               # input clear signal
        self.comb += self.ltc2000.data_in.eq(self.dds.dout)
        self.comb += self.dac_pads.cs.eq(platform.request("user_gpio",0))   #PMOD_0
        self.comb += self.dac_pads.sck.eq(platform.request("user_gpio",1))  #PMOD_1
        self.comb += self.dac_pads.sdi.eq(platform.request("user_gpio",2))  #PMOD_2
        self.comb += platform.request("user_gpio",3).eq(self.dac_pads.sdo)  #PMOD_3
        self.button = platform.request("user_btn_c") 
        self.comb += platform.request("user_led",0).eq(self.button)
        self.comb += self.ltc2000.reset.eq(~self.button)

        # After programming the AC701, the DAC can be configured through the
        # PMOD connector using a Digital Discovery. To see the generated sine
        # use the following script:
        # Start()
        # Write(8,0x03,0x01)
        # Stop()
        # Start()
        # Write(8,0x04,0x0B)
        # Stop()

        self.platform.add_platform_command(
            "set_property CFGBVS VCCO [current_design] \n\
            set_property CONFIG_VOLTAGE 3.3 [current_design]"
        )


def main():
    testsetup = DACTest()
    testsetup.platform.build(testsetup)

if __name__ == "__main__":
    main()
