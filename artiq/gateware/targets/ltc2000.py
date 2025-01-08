from artiq.gateware import rtio
from migen import *
from misoc.interconnect.csr import AutoCSR, CSRStorage
from misoc.interconnect.stream import Endpoint
from artiq.gateware.ltc2000phy import Ltc2000phy
from artiq.gateware.rtio import rtlink
from misoc.cores.duc import PhasedAccu, CosSinGen
from collections import namedtuple

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

class LTC2000DDSModule(Module, AutoCSR):
    """The line data is interpreted as:

        * 16 bit amplitude offset
        * 32 bit amplitude first order derivative
        * 48 bit amplitude second order derivative
        * 48 bit amplitude third order derivative
        * 16 bit phase offset
        * 32 bit frequency word
        * 32 bit chirp
    """

    # def __init__(self):
    def __init__(self, platform, ltc2000_pads):
        self.platform = platform

        self.clear = Signal()
        self.reset = Signal()
        self.data = Signal(16)
        self.ftw = Signal(32)
        self.atw = Signal(32)
        self.ptw = Signal(18)

        # RTIO Interface similar to PDQ
        self.i = Endpoint([("data", 224)])

        z = [Signal(32) for i in range(3)] # phase, dphase, ddphase
        x = [Signal(48) for i in range(4)] # amp, damp, ddamp, dddamp

        self.sync += [
            # za.eq(za + z[1]),
            self.ftw.eq(z[1]),
            self.atw.eq(x[0]),
            self.ptw.eq(z[0]),
            x[0].eq(x[0] + x[1]),
            x[1].eq(x[1] + x[2]),
            x[2].eq(x[2] + x[3]),
            z[1].eq(z[1] + z[2]),
            If(self.i.stb,
                x[0].eq(0),
                x[1].eq(0),
                Cat(x[0][32:], x[1][16:], x[2], x[3], z[0][16:], z[1], z[2] #amp, damp, ddamp, dddamp, phase offset, ftw, chirp
                    ).eq(self.i.payload.raw_bits()),
            )
        ]

        # DDS setup
        self.submodules.dds = ClockDomainsRenamer("sys2x")(PolyphaseDDS(12, 32, 18)) # 12 phases at 200 MHz => 2400 MSPS
        self.comb += [
            self.dds.ftw.eq(self.ftw),  # input frequency tuning word
            self.dds.ptw.eq(self.ptw),  # phase tuning word
            self.dds.clr.eq(self.clear)      # clear signal
        ]

        self.sync.sys2x += [
            self.data.eq(self.dds.dout)
        ]

Phy = namedtuple("Phy", "rtlink probes overrides")

class LTC2000(Module, AutoCSR):

    def __init__(self, platform, ltc2000_pads):
        NUM_OF_DDS = 1
        self.phys = []

        #LTC2000 interface
        self.reset = Signal()
        platform.add_extension(ltc2000_pads)
        self.dac_pads = platform.request("ltc2000")
        platform.add_period_constraint(self.dac_pads.dcko_p, 1.66)
        self.submodules.ltc2000 = Ltc2000phy(self.dac_pads)
        self.comb += self.ltc2000.reset.eq(self.reset)

        trigger_iface = rtlink.Interface(rtlink.OInterface(
            data_width=NUM_OF_DDS,
            enable_replace=False))
        self.phys.append(Phy(trigger_iface, [], []))

        clear = Signal()

        for idx in range(NUM_OF_DDS):
            tone = LTC2000DDSModule(platform, ltc2000_pads)
            # self.comb += [
            #     tone.clear.eq(self.cfg.clr[idx]),
            #     tone.gain.eq(self.cfg.gain[idx]),
            # ]

            rtl_iface = rtlink.Interface(rtlink.OInterface(
                data_width=16, address_width=4))

            array = Array(tone.i.data[wi: wi+16] for wi in range(0, len(tone.i.data), 16))

            self.sync.rio += [
                tone.i.stb.eq(trigger_iface.o.data[idx] & trigger_iface.o.stb),
                If(rtl_iface.o.stb,
                    array[rtl_iface.o.address].eq(rtl_iface.o.data),
                ),
            ]

            self.phys.append(Phy(rtl_iface, [], []))

            self.submodules += tone
            self.sync.sys2x += [
                self.ltc2000.data_in.eq(tone.data)
            ]

        # self.submodules.cfg = Config()
        # cfg_rtl_iface = rtlink.Interface(
        #     rtlink.OInterface(
        #         data_width=len(self.cfg.i.data),
        #         address_width=len(self.cfg.i.addr),
        #         enable_replace=False,
        #     ),
        #     rtlink.IInterface(
        #         data_width=len(self.cfg.o.data),
        #     ),
        # )

        # self.comb += [
        #     self.cfg.i.stb.eq(cfg_rtl_iface.o.stb),
        #     self.cfg.i.addr.eq(cfg_rtl_iface.o.address),
        #     self.cfg.i.data.eq(cfg_rtl_iface.o.data),
        #     cfg_rtl_iface.i.stb.eq(self.cfg.o.stb),
        #     cfg_rtl_iface.i.data.eq(self.cfg.o.data),
        # ]
        # self.phys.append(Phy(cfg_rtl_iface, [], []))
