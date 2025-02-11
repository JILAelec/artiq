from artiq.gateware import rtio
from migen import *
from misoc.interconnect.csr import AutoCSR, CSRStorage
from misoc.interconnect.stream import Endpoint
from artiq.gateware.ltc2000phy import Ltc2000phy
from artiq.gateware.rtio import rtlink
from misoc.cores.duc import PhasedAccu, CosSinGen, saturate
from collections import namedtuple
from sumandscale import SumAndScale

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
        # #Output
        self.amplitude = Signal(16)
        # self.dout = Signal((16*NPHASES*2, True))
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

        # Make amplitude accessible
        self.comb += self.amplitude.eq(x[0][32:])

        # DDS setup
        self.submodules.dds = DoubleDataRateDDS(NPHASES, 32, 18) # 12 phases at 200 MHz => 2400 MSPS, output updated at 100 MHz
        self.comb += [
            self.dds.ftw.eq(self.ftw),  # input frequency tuning word
            self.dds.ptw.eq(self.ptw),  # phase tuning word
            self.dds.clr.eq(self.clear) # clear signal
        ]

        # # Multiply DDS output with amplitude
        # for i in range(NPHASES*2):
        #     scaled = Signal((32, True))
        #     self.sync += [
        #         scaled.eq(self.dds.dout[i*16:(i+1)*16] * x[0][32:]),
        #         self.dout[i*16:(i+1)*16].eq(scaled >> 15)  # Scale back to 16 bits
        #     ]

class LTC2000DataSynth(Module, AutoCSR):
    def __init__(self, NUM_OF_DDS, NPHASES):
        self.amplitudes = Array([[Signal(16, name=f"amplitudes_{i}_{j}") for i in range(NPHASES)] for j in range(NUM_OF_DDS)])
        self.data_in = Array([[Signal(16, name=f"data_in_{i}_{j}") for i in range(NPHASES)] for j in range(NUM_OF_DDS)])
        self.ios = []

        # Sum all DDS outputs with saturation
        self.summers = [SumAndScale() for _ in range(NPHASES)]
        for idx, summer in enumerate(self.summers):
            setattr(self.submodules, f"summer{idx}", summer)

        # Add and saturate all of our samples at their respective phases
        for i in range(NPHASES):
            for j in range(NUM_OF_DDS):
                self.ios.append(self.amplitudes[j][i])
                self.ios.append(self.data_in[j][i])
                self.comb += [
                    self.summers[i].inputs[j].eq(self.data_in[j][i]),
                    self.summers[i].amplitudes[j].eq(self.amplitudes[j][i]),
                ]

Phy = namedtuple("Phy", "rtlink probes overrides name")

class LTC2000(Module, AutoCSR):

    def __init__(self, platform, ltc2000_pads):
        NUM_OF_DDS = 4
        NPHASES = 24

        self.submodules.ltc2000datasynth = LTC2000DataSynth(NUM_OF_DDS, NPHASES)

        self.tones = [LTC2000DDSModule() for _ in range(NUM_OF_DDS)]
        for idx, tone in enumerate(self.tones):
            setattr(self.submodules, f"tone{idx}", tone)

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

        # Connect DDS to sum and scale
        for i in range(NPHASES):
            for j in range(NUM_OF_DDS):
                self.comb += self.ltc2000datasynth.data_in[j][i].eq(self.tones[j].dds.dout[i*16:(i+1)*16])
                self.comb += self.ltc2000datasynth.amplitudes[j][i].eq(self.tones[j].amplitude)

        # Connect to DAC
        for i in range(NPHASES):
            self.sync += self.ltc2000.data[i*16:(i+1)*16].eq(self.ltc2000datasynth.summers[i].output)

        TESTDAC = True #turn on to output 12.5 MHz test sine wave
        if TESTDAC:
            # Sample data array for testing (8 * 24 samples)
            sample_data = [
                0x0000, 0x0430, 0x085f, 0x0c8c, 0x10b5, 0x14da, 0x18f9, 0x1d11, 0x2121, 0x2528,
                0x2925, 0x2d16, 0x30fb, 0x34d3, 0x389c, 0x3c56, 0x3fff, 0x4397, 0x471c, 0x4a8e,
                0x4deb, 0x5133, 0x5465, 0x577f, 0x5a82, 0x5d6b, 0x603c, 0x62f1, 0x658c, 0x680b,
                0x6a6d, 0x6cb2, 0x6ed9, 0x70e2, 0x72cc, 0x7496, 0x7641, 0x77cb, 0x7934, 0x7a7c,
                0x7ba2, 0x7ca7, 0x7d89, 0x7e49, 0x7ee7, 0x7f61, 0x7fb9, 0x7fed, 0x7fff, 0x7fed,
                0x7fb9, 0x7f61, 0x7ee7, 0x7e49, 0x7d89, 0x7ca7, 0x7ba2, 0x7a7c, 0x7934, 0x77cb,
                0x7641, 0x7496, 0x72cc, 0x70e2, 0x6ed9, 0x6cb2, 0x6a6d, 0x680b, 0x658c, 0x62f1,
                0x603c, 0x5d6b, 0x5a82, 0x577f, 0x5465, 0x5133, 0x4deb, 0x4a8e, 0x471c, 0x4397,
                0x3fff, 0x3c56, 0x389c, 0x34d3, 0x30fb, 0x2d16, 0x2925, 0x2528, 0x2121, 0x1d11,
                0x18f9, 0x14da, 0x10b5, 0x0c8c, 0x085f, 0x0430, 0x0000, 0xfbd0, 0xf7a1, 0xf374,
                0xef4b, 0xeb26, 0xe707, 0xe2ef, 0xdedf, 0xdad8, 0xd6db, 0xd2ea, 0xcf05, 0xcb2d,
                0xc764, 0xc3aa, 0xc001, 0xbc69, 0xb8e4, 0xb572, 0xb215, 0xaecd, 0xab9b, 0xa881,
                0xa57e, 0xa295, 0x9fc4, 0x9d0f, 0x9a74, 0x97f5, 0x9593, 0x934e, 0x9127, 0x8f1e,
                0x8d34, 0x8b6a, 0x89bf, 0x8835, 0x86cc, 0x8584, 0x845e, 0x8359, 0x8277, 0x81b7,
                0x8119, 0x809f, 0x8047, 0x8013, 0x8001, 0x8013, 0x8047, 0x809f, 0x8119, 0x81b7,
                0x8277, 0x8359, 0x845e, 0x8584, 0x86cc, 0x8835, 0x89bf, 0x8b6a, 0x8d34, 0x8f1e,
                0x9127, 0x934e, 0x9593, 0x97f5, 0x9a74, 0x9d0f, 0x9fc4, 0xa295, 0xa57e, 0xa881,
                0xab9b, 0xaecd, 0xb215, 0xb572, 0xb8e4, 0xbc69, 0xc000, 0xc3aa, 0xc764, 0xcb2d,
                0xcf05, 0xd2ea, 0xd6db, 0xdad8, 0xdedf, 0xe2ef, 0xe707, 0xeb26, 0xef4b, 0xf374,
                0xf7a1, 0xfbd0
            ]

            # Convert sample data to Signals
            sample_signals = Array(Signal(16, reset=sample) for sample in sample_data)

            # Counter to cycle through the sets
            sample_counter = Signal(3)

            # Create a multiplexer for the samples
            selected_samples = Array(Cat(*sample_signals[i*24:(i+1)*24]) for i in range(8))

            # Apply test data to LTC2000
            self.sync += [
                self.ltc2000.data.eq(selected_samples[sample_counter]),
                sample_counter.eq(sample_counter + 1)
            ]

        self.phys.append(Phy(trigger_iface, [], [], 'trigger_iface'))
        self.phys.append(Phy(clear_iface, [], [], 'clear_iface'))
        self.phys.append(Phy(reset_iface, [], [], 'reset_iface'))
        self.phys.append(Phy(gain_iface, [], [], 'gain_iface'))

