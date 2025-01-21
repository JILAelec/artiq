from artiq.gateware import rtio
from migen import *
from misoc.interconnect.csr import AutoCSR, CSRStorage
from misoc.interconnect.stream import Endpoint
from artiq.gateware.ltc2000phy import Ltc2000phy
from artiq.gateware.rtio import rtlink
from misoc.cores.duc import PhasedAccu, CosSinGen, saturate
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

class DoubleDataRateDDS(Module):
    """Composite DDS running at twice the system clock rate.
    """
    def __init__(self, n, fwidth, pwidth, z=18, x=15, zl=9, xd=4, backoff=None, share_lut=None):
        self.ftw  = Signal(fwidth)   # input frequency tuning word
        self.ptw  = Signal(pwidth)   # input phase tuning word
        self.clr  = Signal()         # input clear signal
        self.dout = Signal((x+1)*n*2)  # output data

        ###

        paccu = ClockDomainsRenamer("sys2x")(PhasedAccu(n, fwidth, pwidth)) # Running this at 2x clock speed
        self.comb += [
            paccu.clr.eq(self.clr),
            paccu.f.eq(self.ftw),
            paccu.p.eq(self.ptw),
        ]
        self.submodules.paccu = paccu
        self.ddss = [ClockDomainsRenamer("sys2x")(CosSinGen()) for _ in range(n)]
        counter = Signal()
        dout2x = Signal((x+1)*n*2)  # output data modified in 2x domain
        for idx, dds in enumerate(self.ddss):
            setattr(self.submodules, f"dds{idx}", dds)
            self.comb += dds.z.eq(paccu.z[idx])

            self.sync.sys2x += [
                If(counter,
                    dout2x[idx*16:(idx+1)*16].eq(dds.y)
                ).Else(
                    dout2x[(idx+n)*16:(idx+n+1)*16].eq(dds.y)
                ),
                counter.eq(~counter)
            ]
        self.sync += [
            If(~counter,
                # Output the full dout array in sys domain
                self.dout.eq(dout2x)
            )
        ]


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
    def __init__(self):
        NPHASES = 12
        self.clear = Signal()
        self.ftw = Signal(32)
        self.atw = Signal(32)
        self.ptw = Signal(18)
        #Output
        self.dout = Signal((16*NPHASES*2, True))
        # gain for static magnitude scaling
        self.gain = Signal(16)

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
        self.submodules.dds = DoubleDataRateDDS(NPHASES, 32, 18) # 12 phases at 200 MHz => 2400 MSPS, output updated at 100 MHz
        self.comb += [
            self.dds.ftw.eq(self.ftw),  # input frequency tuning word
            self.dds.ptw.eq(self.ptw),  # phase tuning word
            self.dds.clr.eq(self.clear) # clear signal
        ]

        # Multiply DDS output with amplitude
        for i in range(NPHASES*2):
            scaled = Signal((32, True))
            self.sync += [
                scaled.eq(self.dds.dout[i*16:(i+1)*16] * x[0][32:]),
                self.dout[i*16:(i+1)*16].eq(scaled >> 15)  # Scale back to 16 bits
            ]


Phy = namedtuple("Phy", "rtlink probes overrides name")

class LTC2000(Module, AutoCSR):

    def __init__(self, platform, ltc2000_pads):
        NUM_OF_DDS = 4
        self.phys = []

        #LTC2000 interface
        platform.add_extension(ltc2000_pads)
        self.dac_pads = platform.request("ltc2000")
        # platform.add_period_constraint(self.dac_pads.dcko_p, 1.66)
        self.submodules.ltc2000 = Ltc2000phy(self.dac_pads)

        clear = Signal(NUM_OF_DDS)
        reset = Signal()
        trigger = Signal(NUM_OF_DDS)
        self.comb += self.ltc2000.reset.eq(reset)

        self.tones = [LTC2000DDSModule() for _ in range(NUM_OF_DDS)]

        gain_iface = rtlink.Interface(rtlink.OInterface(
            data_width=16,
            address_width=4,
            enable_replace=False
        ))

        tone_gains = Array([tone.gain for tone in self.tones])
        self.sync.rio += [
            If(gain_iface.o.stb,
                tone_gains[gain_iface.o.address].eq(gain_iface.o.data)
            )
        ]

        clear_iface = rtlink.Interface(rtlink.OInterface(
            data_width=NUM_OF_DDS,
            enable_replace=False
        ))

        self.sync.rio += [
            If(clear_iface.o.stb,
                clear.eq(clear_iface.o.data)
            )
        ]

        trigger_iface = rtlink.Interface(rtlink.OInterface(
            data_width=NUM_OF_DDS,
            enable_replace=False))

        self.sync.rio += [
            If(trigger_iface.o.stb,
                trigger.eq(trigger_iface.o.data)
            )
        ]

        reset_iface = rtlink.Interface(rtlink.OInterface(
            data_width=1,
            enable_replace=False))

        self.sync.rio += [
            If(reset_iface.o.stb,
                reset.eq(reset_iface.o.data)
            )
        ]

        for idx, tone in enumerate(self.tones):
            setattr(self.submodules, f"tone{idx}", tone)
            self.comb += [
                tone.clear.eq(clear[idx]),
            ]

            rtl_iface = rtlink.Interface(rtlink.OInterface(
                data_width=16, address_width=4))

            array = Array(tone.i.data[wi: wi+16] for wi in range(0, len(tone.i.data), 16))

            self.sync.rio += [
                tone.i.stb.eq(trigger_iface.o.data[idx] & trigger_iface.o.stb),
                If(rtl_iface.o.stb,
                    array[rtl_iface.o.address].eq(rtl_iface.o.data),
                ),
            ]

            self.phys.append(Phy(rtl_iface, [], [], 'rtl_iface'))

        NPHASES = 24

        # Sum all DDS outputs with saturation
        dds_sum = Signal((18*NPHASES, True))  # Extra bits for summing
        final_output = Signal((16*NPHASES, True))

        # First add all channels
        for i in range(NPHASES):
            # # Extract and sum samples from each tone
            # self.comb += dds_sum[i*17:(i+1)*17].eq(
            #     (self.tones[0].dout[i*16:(i+1)*16] +
            #      self.tones[1].dout[i*16:(i+1)*16] +
            #      self.tones[2].dout[i*16:(i+1)*16] +
            #      self.tones[3].dout[i*16:(i+1)*16])
            # )

            # # Saturate each summed sample
            # self.sync += saturate(
            #     final_output[i*16:(i+1)*16],
            #     dds_sum[i*17:(i+1)*17]
            # )

            # Sum without saturation for now, saturation has bug
            self.sync += final_output[i*16:(i+1)*16].eq(
                self.tones[0].dout[i*16:(i+1)*16] +
                self.tones[1].dout[i*16:(i+1)*16] +
                self.tones[2].dout[i*16:(i+1)*16] +
                self.tones[3].dout[i*16:(i+1)*16]
            )

        # Connect to DAC
        self.sync += self.ltc2000.data.eq(final_output)

        TESTDAC = False #turn on to output 100 MHz test sine wave
        if TESTDAC:
            # Sample data array for testing
            sample_data = [
                0x0000, 0x2120, 0x3fff, 0x5a81, 0x6ed9, 0x7ba2, 0x7fff, 0x7ba2, 0x6ed9, 0x5a81,
                0x3fff, 0x2120, 0x0000, 0xdee0, 0xc001, 0xa57f, 0x9127, 0x845e, 0x8001, 0x845e,
                0x9127, 0xa57f, 0xc001, 0xdee0
            ]

            # Convert sample data to Signals
            sample_signals = Array(Signal(16, reset=sample) for sample in sample_data)

            #Apply test data to LTC2000
            self.sync += [
                self.ltc2000.data.eq(Cat(*sample_signals))
            ]

        self.phys.append(Phy(trigger_iface, [], [], 'trigger_iface'))
        self.phys.append(Phy(clear_iface, [], [], 'clear_iface'))
        self.phys.append(Phy(reset_iface, [], [], 'reset_iface'))
        self.phys.append(Phy(gain_iface, [], [], 'gain_iface'))

