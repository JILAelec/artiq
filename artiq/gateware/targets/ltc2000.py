from migen import *
from misoc.interconnect.csr import *
from artiq.language.core import kernel, delay
from artiq.gateware.ltc2000phy import Ltc2000phy
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
    def __init__(self, platform, ltc2000_pads):
        self.platform = platform
        self.rtio_channels = []

        # Define CSRs
        self.ftw = CSRStorage(32, name="ftw")  # Frequency Tuning Word
        self.ptw = CSRStorage(32, name="ptw")  # Phase Tuning Word
        self.clr = CSRStorage(1, name="clr")   # Clear Signal
        self.reset = CSRStorage(1, name="ltc2000_reset")  # Reset Signal

        # Add RTIO PHY
        self.rtio_phy = rtio.phy.SimpleInterface(
            rtio.channel.Interface(
                rtio.channel.OInterface(data_width=32, address_width=4),
                rtio.channel.IInterface(data_width=32, timestamped=False)
            )
        )

        # RTIO to CSR bridge
        self.sync += [
            If(self.rtio_phy.o.stb,
                Case(self.rtio_phy.o.address[0:2], {
                    0: self.ftw.storage.eq(self.rtio_phy.o.data),
                    1: self.pow.storage.eq(self.rtio_phy.o.data),
                    2: self.asf.storage.eq(self.rtio_phy.o.data),
                    3: self.rtio_phy.i.data.eq(self.status.status),
                }),
                self.rtio_phy.i.stb.eq(1)
            )
        ]

        # LTC2000 setup
        platform.add_extension(ltc2000_pads)
        self.dac_pads = platform.request("ltc2000")
        platform.add_period_constraint(self.dac_pads.dcko_p, 1.66)
        self.submodules.ltc2000 = Ltc2000phy(self.dac_pads)

        # DDS setup
        self.submodules.dds = PolyphaseDDS(16, 32, 18)
        self.comb += [
            self.dds.ftw.eq(self.ftw.storage),  # input frequency tuning word
            self.dds.ptw.eq(self.ptw.storage),  # phase tuning word
            self.dds.clr.eq(self.clr.storage),  # clear signal
            self.ltc2000.data_in.eq(self.dds.dout)
        ]

        # Reset signal with CSR and button
        self.button = platform.request("user_btn_c")
        self.comb += self.ltc2000.reset.eq(self.reset.storage | ~self.button)

    @kernel
    def reset(self):
        self.reset.write(1)
        delay(1*us)  # Adjust the delay as needed
        self.reset.write(0)