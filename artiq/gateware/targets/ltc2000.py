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
    """Composite DDS with sub-DDSs synthesizing
       individual phases to increase fmax.
    """
    def __init__(self, n, fwidth, pwidth, z=18, x=15, zl=9, xd=4, backoff=None, share_lut=None):
        self.ftw  = Signal(fwidth)
        self.ptw  = Signal(pwidth)
        self.clr  = Signal()
        self.dout = Signal((x+1)*n)

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
        self.ftw  = Signal(fwidth)
        self.ptw  = Signal(pwidth)
        self.clr  = Signal()
        self.dout = Signal((x+1)*n*2)

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
                    dout2x[idx*16:(idx+1)*16].eq(dds.x)
                ).Else(
                    dout2x[(idx+n)*16:(idx+n+1)*16].eq(dds.x)
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
        self.amplitude = Signal(16)
        self.gain = Signal(16)
        self.shift = Signal(4)
        self.shift_counter = Signal(16) #need to count to 2**shift - 1
        self.shift_stb = Signal()
        self.reserved = Signal(12) # for future use

        self.i = Endpoint([("data", 240)])

        self.comb += [
            self.shift_stb.eq((self.shift == 0) |
                             (self.shift_counter == (1 << self.shift) - 1)) # power of two for strobing
        ]
        self.sync += [
            If(self.shift == 0,
                self.shift_counter.eq(0)
            ).Elif(self.shift_counter == (1 << self.shift) - 1,
                self.shift_counter.eq(0)
            ).Else(
                self.shift_counter.eq(self.shift_counter + 1)
            )
        ]

        z = [Signal(32) for i in range(3)] # phase, dphase, ddphase
        x = [Signal(48) for i in range(4)] # amp, damp, ddamp, dddamp

        self.sync += [
            self.ftw.eq(z[1]),
            self.atw.eq(x[0]),
            self.ptw.eq(z[0]),

            #using shift here as a divider
            If(self.shift_stb,
                x[0].eq(x[0] + x[1]),
                x[1].eq(x[1] + x[2]),
                x[2].eq(x[2] + x[3]),
                z[1].eq(z[1] + z[2]),
            ),

            If(self.i.stb,
                x[0].eq(0),
                x[1].eq(0),
                Cat(x[0][32:],           # amp offset (16 bits)
                    x[1][16:],           # damp (32 bits)
                    x[2],                # ddamp (48 bits)
                    x[3],                # dddamp (48 bits)
                    z[0][16:],           # phase offset (16 bits)
                    z[1],                # ftw (32 bits)
                    z[2],                # chirp (32 bits)
                    self.reserved,       # reserved (12 bits)
                    self.shift,          # shift (4 UPPER bits)
                ).eq(self.i.payload.raw_bits()),
                self.shift_counter.eq(0),
            )
        ]

        self.comb += self.amplitude.eq(x[0][32:])

        self.submodules.dds = DoubleDataRateDDS(NPHASES, 32, 18) # 12 phases at 200 MHz => 2400 MSPS, output updated at 100 MHz
        self.comb += [
            self.dds.ftw.eq(self.ftw),
            self.dds.ptw.eq(self.ptw),
            self.dds.clr.eq(self.clear)
        ]


class LTC2000DataSynth(Module, AutoCSR):
    def __init__(self, NUM_OF_DDS, NPHASES):
        self.amplitudes = Array([[Signal(16, name=f"amplitudes_{i}_{j}") for i in range(NPHASES)] for j in range(NUM_OF_DDS)])
        self.data_in = Array([[Signal(16, name=f"data_in_{i}_{j}") for i in range(NPHASES)] for j in range(NUM_OF_DDS)])
        self.ios = []

        self.summers = [SumAndScale() for _ in range(NPHASES)]
        for idx, summer in enumerate(self.summers):
            setattr(self.submodules, f"summer{idx}", summer)

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

        platform.add_extension(ltc2000_pads)
        self.dac_pads = platform.request("ltc2000")
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

        for i in range(NPHASES):
            for j in range(NUM_OF_DDS):
                self.comb += self.ltc2000datasynth.data_in[j][i].eq(self.tones[j].dds.dout[i*16:(i+1)*16])
                self.comb += self.ltc2000datasynth.amplitudes[j][i].eq(self.tones[j].amplitude)

        for i in range(NPHASES):
            self.sync += self.ltc2000.data[i*16:(i+1)*16].eq(self.ltc2000datasynth.summers[i].output)

        self.phys.append(Phy(trigger_iface, [], [], 'trigger_iface'))
        self.phys.append(Phy(clear_iface, [], [], 'clear_iface'))
        self.phys.append(Phy(reset_iface, [], [], 'reset_iface'))
        self.phys.append(Phy(gain_iface, [], [], 'gain_iface'))

### test below here

# Replace everything after the LTC2000 class with this:

class LTC2000DDSModuleTest(Module):
    """Simplified version that ONLY tests coefficient processing - no DDS"""

    def __init__(self):
        self.clear = Signal()
        self.ftw = Signal(32)
        self.atw = Signal(32)
        self.ptw = Signal(18)
        self.amplitude = Signal(16)
        self.gain = Signal(16)

        self.i = Endpoint([("data", 224)])

        z = [Signal(32) for i in range(3)] # phase, dphase, ddphase
        x = [Signal(48) for i in range(4)] # amp, damp, ddamp, dddamp

        # Expose z and x for testing
        self.z = z
        self.x = x

        self.sync += [
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
                Cat(x[0][32:], x[1][16:], x[2], x[3], z[0][16:], z[1], z[2]).eq(self.i.payload.raw_bits()),
            )
        ]

        self.comb += self.amplitude.eq(x[0][32:])

def test_coefficient_processing():
    """Test coefficient processing and monitor amplitude over time"""

    # Get total cycles from user
    try:
        total_cycles = int(input("Enter total number of cycles: "))
    except ValueError:
        print("Invalid input, using default: 100 cycles")
        total_cycles = 100

    # Calculate interval for 20 intermediate points + final point
    interval = max(1, total_cycles // 20)
    output_points = [i * interval for i in range(21)]  # 0, interval, 2*interval, ..., 20*interval

    # Make sure the last point is exactly at the requested cycle
    if output_points[-1] != total_cycles:
        output_points[-1] = total_cycles

    def tb_process(dut):
        # Load initial coefficients
        test_case = {
            'amp': 14251,
            'damp': 429,
            'ddamp': 4,
            'dddamp': 0x0,
            'phase_offset': 0x0,
            'ftw': 0x0,
            'chirp': 0x0
        }

        print("Loading initial coefficients:")
        for key, value in test_case.items():
            print(f"  {key:12}: 0x{value:x}")

        # Pack and load coefficients
        packed_data = (
            test_case['amp'] |                          # [15:0]
            (test_case['damp'] << 16) |                 # [47:16]
            (test_case['ddamp'] << 48) |                # [95:48]
            (test_case['dddamp'] << 96) |               # [143:96]
            (test_case['phase_offset'] << 144) |        # [159:144]
            (test_case['ftw'] << 160) |                 # [191:160]
            (test_case['chirp'] << 192)                 # [223:192]
        )

        # Apply coefficients
        yield dut.i.payload.data.eq(packed_data)
        yield dut.i.stb.eq(1)
        yield
        yield dut.i.stb.eq(0)

        print(f"\nMonitoring amplitude over {total_cycles} cycles (20 intermediate points + final):")

        # Calculate the width needed for cycle numbers
        cycle_width = max(len(str(total_cycles)), 5)  # 5 is length of "Cycle"

        # Simple right-justified format without vertical lines
        print(f"{'Cycle':>{cycle_width}}   Amplitude (hex)   Amplitude (dec)")
        print(f"{'-' * cycle_width}   ---------------   ---------------")

        output_index = 0

        # Monitor amplitude for total_cycles
        for cycle in range(total_cycles + 1):
            # Check if we should output at this cycle
            if output_index < len(output_points) and cycle == output_points[output_index]:
                amplitude = yield dut.amplitude
                print(f"{cycle:>{cycle_width}d}        0x{amplitude:04x}            {amplitude:6d}")
                output_index += 1

            # Advance one cycle (except on the last iteration)
            if cycle < total_cycles:
                yield

    # Run simulation
    dut = LTC2000DDSModuleTest()

    from migen.sim import Simulator

    def clock():
        # Run for enough cycles to complete the test
        for _ in range(total_cycles + 10):  # Extra cycles for setup
            yield

    print("Testing coefficient processing with amplitude monitoring...")
    sim = Simulator(dut, [tb_process(dut), clock()])
    sim.run()
    print("Done!")

if __name__ == "__main__":
    test_coefficient_processing()
