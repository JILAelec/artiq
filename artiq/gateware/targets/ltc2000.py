from artiq.gateware import rtio
from migen import *
from misoc.interconnect.csr import AutoCSR, CSRStorage
from misoc.interconnect.stream import Endpoint
from artiq.gateware.ltc2000phy import Ltc2000phy
from artiq.gateware.rtio import rtlink
from misoc.cores.duc import PhasedAccu, CosSinGen

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

    FTW_ADDR = 0
    ATW_ADDR = 1
    PTW_ADDR = 2
    CLR_ADDR = 3
    RST_ADDR = 4

    def __init__(self):
    # def __init__(self, platform, ltc2000_pads):
        # self.platform = platform
        self.rtio_channels = []

        # Define CSRs
        self.ftw = CSRStorage(32, reset=0x05F5E100, name="ftw")  # Frequency Tuning Word - 2400/2^32*FTW MHz -- 55.88 MHz
        self.atw = CSRStorage(32, reset=0x00000000, name="atw")  # Amplitude Tuning Word
        self.ptw = CSRStorage(32, reset=0x00000000, name="ptw")  # Phase Tuning Word
        self.clr = CSRStorage(1, reset=0x0, name="clr")          # Clear Signal
        self.reset = CSRStorage(1, reset=0x0, name="ltc2000_reset")  # Reset Signal

        # RTIO Interface similar to PDQ
        self.i = Endpoint([("data", 224)])

        clear = Signal()

        za = Signal(32)
        z = [Signal(32) for i in range(3)] # phase, dphase, ddphase
        x = [Signal(48) for i in range(4)] # amp, damp, ddamp, dddamp

        # self.comb += [
        #     self.cordic.xi.eq(x[0][32:]), #frequency
        #     self.cordic.zi.eq(za[16:] + z[0][16:]), #phase
        #     self.data.eq(self.cordic.xo),
        # ]

        self.sync += [
            za.eq(za + z[1]),
            x[0].eq(x[0] + x[1]),
            x[1].eq(x[1] + x[2]),
            x[2].eq(x[2] + x[3]),
            z[1].eq(z[1] + z[2]),
            If(self.i.stb,
                x[0].eq(0),
                x[1].eq(0),
                Cat(x[0][32:], x[1][16:], x[2], x[3], z[0][16:], z[1], z[2] #amp, damp, ddamp, dddamp, phase offset, ftw, chirp
                    ).eq(self.i.payload.raw_bits()),
                If(clear,
                    za.eq(0),
                )
            )
        ]

        # # Add RTIO PHY
        # self.rtlink = rtlink.Interface(
        #     rtlink.OInterface(
        #         data_width=32,
        #         address_width=4,
        #         enable_replace=False),
        #     rtlink.IInterface(
        #         data_width=32,
        #         timestamped=False)
        # )

        # # Define previous_data and previous_address signals
        # previous_data = Signal.like(self.rtlink.o.data)
        # previous_address = Signal.like(self.rtlink.o.address)

        # # RTIO to CSR bridge
        # self.sync += [
        #     If(self.rtlink.o.stb,
        #         # Check if data or address has changed
        #         If((self.rtlink.o.data != previous_data) | (self.rtlink.o.address != previous_address),
        #             Case(self.rtlink.o.address[0:2], {
        #                 self.FTW_ADDR: self.ftw.storage_full.eq(self.rtlink.o.data),
        #                 self.ATW_ADDR: self.atw.storage_full.eq(self.rtlink.o.data),
        #                 self.PTW_ADDR: self.ptw.storage_full.eq(self.rtlink.o.data),
        #                 self.CLR_ADDR: self.clr.storage_full.eq(self.rtlink.o.data),
        #                 self.RST_ADDR: self.reset.storage_full.eq(self.rtlink.o.data)
        #             }),
        #             self.rtlink.i.stb.eq(1)
        #         ),
        #         # Update previous_data and previous_address with the current values
        #         previous_data.eq(self.rtlink.o.data),
        #         previous_address.eq(self.rtlink.o.address)
        #     )
        # ]

        # LTC2000 setup
        # platform.add_extension(ltc2000_pads)
        # self.dac_pads = platform.request("ltc2000")
        # platform.add_period_constraint(self.dac_pads.dcko_p, 1.66)
        # self.submodules.ltc2000 = Ltc2000phy(self.dac_pads)

        # DDS setup
        self.submodules.dds = ClockDomainsRenamer("sys2x")(PolyphaseDDS(12, 32, 18)) # 12 phases at 200 MHz => 2400 MSPS
        self.sync += [
            self.dds.ftw.eq(self.ftw.storage_full),  # input frequency tuning word
            self.dds.ptw.eq(self.ptw.storage_full),  # phase tuning word
            self.dds.clr.eq(self.clr.storage_full)   # clear signal
        ]

        self.sync.sys2x += [
            self.ltc2000.data_in.eq(self.dds.dout)
        ]

        # Reset signal with CSR and button
        self.button = platform.request("user_btn_c")
        self.comb += self.ltc2000.reset.eq(self.reset.storage_full | self.button)