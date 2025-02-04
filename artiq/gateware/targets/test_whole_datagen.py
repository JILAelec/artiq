from migen import *
from migen.fhdl import verilog
from migen.sim import run_simulation
from ltc2000 import LTC2000DataSynth, PolyphaseDDS, DoubleDataRateDDS

class TestRig(Module):
    def __init__(self):
        NUM_OF_DDS = 2
        NPHASES = 12
        AMPLITUDES = [0x4000,0x4000]

        self.submodules.ddsa = PolyphaseDDS(12,32,18)
        self.submodules.ddsb = PolyphaseDDS(12,32,18)
        self.submodules.ltc2000datasynth = LTC2000DataSynth(NUM_OF_DDS, NPHASES)

        for i in range(NPHASES):
            self.comb += self.ltc2000datasynth.data_in[0][i].eq(self.ddsa.dout[i*16:(i+1)*16])
            self.comb += self.ltc2000datasynth.amplitudes[0][i].eq(AMPLITUDES[0])
            self.comb += self.ltc2000datasynth.data_in[1][i].eq(self.ddsb.dout[i*16:(i+1)*16])
            self.comb += self.ltc2000datasynth.amplitudes[1][i].eq(AMPLITUDES[1])

def test_bench():
    dds = PolyphaseDDS(12, 32, 18) # 12 phases at 200 MHz => 2400 MSPS, output updated at 100 MHz
    tr = TestRig()

    def simple_dds_test():
        """
        Just making sure we can see the output from a single DDS core before we continue on.
        """
        data = [0 for _ in range(12)]

        yield dds.ftw.eq(0x01000000)
        yield dds.ptw.eq(0)
        yield dds.clr.eq(1)
        yield
        yield dds.clr.eq(0)
        for i in range(30):
            yield
            for i in range(12):
                data[i] = yield dds.dout[i*16:(i+1)*16]
                print(f'{data[i]:04x} ', end='')
            print('')

    def sdr_test(testrig):
        """
        Test for data generation system at single data rate. This is a placeholder until we
        got the double data rate scenario worked out as that's what we actually use in the
        experiment.
        """
        data = [0 for _ in range(12)]

        ddss = [testrig.ddsa, testrig.ddsb]

        for dds in ddss:
            yield dds.ftw.eq(0x01000000)
            yield dds.ptw.eq(0)
            yield dds.clr.eq(1)
        yield
        for dds in ddss:
            yield dds.clr.eq(0)
        for i in range(30):
            yield
            for dds in ddss:
                for i in range(12):
                    data[i] = yield dds.dout[i*16:(i+1)*16]
                    print(f'{data[i]:04x} ', end='')
                print('     ', end='')
            print('')
            for i in range(12):
                printme = yield testrig.ltc2000datasynth.summers[i].output
                print(f'{printme:04x} ', end='')
            print('')
            print('')

# for i in range(NPHASES):
#             for j in range(NUM_OF_DDS):
#                 self.comb += self.ltc2000datasynth.data_in[j][i].eq(self.tones[j].dds.dout[i*16:(i+1)*16])
#                 self.comb += self.ltc2000datasynth.amplitudes[j][i].eq(self.tones[j].amplitude)

#         # Connect to DAC
#         for i in range(NPHASES):
#             self.sync += self.ltc2000.data[i*16:(i+1)*16].eq(self.ltc2000datasynth.summers[i].output)


    # def test_with_amplitude(amp_value, active_channel=None):
    #     """
    #     Run test with given amplitude. If active_channel is specified (0-3),
    #     only that channel will receive input while others are set to 0.
    #     """
    #     LATENCY = 2
    #     test_type = "single channel" if active_channel is not None else "all channels"
    #     print(f"\n=== Testing with amplitude {amp_value:04x} ({test_type}) ===")

    #     # Set amplitudes
    #     for dds in range(4):
    #         for phase in range(24):
    #             yield dut.amplitudes[dds][phase].eq(amp_value)
    #     yield

    #     input_history = []

    #     for cycle in range(10):
    #         if cycle < len(sample_data):
    #             input_value = sample_data[cycle]
    #             input_history.append(input_value)
    #             print(f"\nCycle {cycle}: Setting input value {input_value:04x}")

    #             # Set inputs based on test mode
    #             for dds in range(4):
    #                 for phase in range(24):
    #                     if active_channel is None or dds == active_channel:
    #                         yield dut.data_in[dds][phase].eq(input_value)
    #                     else:
    #                         yield dut.data_in[dds][phase].eq(0)

    #         yield

    #         if cycle >= LATENCY:
    #             output = (yield dut.summers[0].output)
    #             expected_input = input_history[cycle-LATENCY]

    #             # Calculate expected output based on test mode
    #             multiplier = 4 if active_channel is None else 1
    #             product = (expected_input * amp_value * multiplier)
    #             expected_output = product >> 16

    #             # Apply saturation
    #             if expected_output > 32767:
    #                 expected_output = 32767
    #             elif expected_output < -32768:
    #                 expected_output = -32768
    #             expected_output = expected_output & 0xFFFF

    #             print(f"Phase 0 Output: {output & 0xFFFF:04x} (from input {expected_input:04x})")
    #             print(f"Expected Output: {expected_output:04x} with scaling {amp_value:04x}")

    #             if abs((output & 0xFFFF) - expected_output) > 2:
    #                 print(f"WARNING: Output differs from expected!")

    def tb_generator():
        yield from simple_dds_test()

        for _ in range(2): yield

        yield from sdr_test(tr)

        # test_amplitudes = [
        #     0x2000,  # 1/8 scale
        #     0x4000,  # 1/4 scale
        #     0x8000,  # 1/2 scale
        #     0xFFFF   # Full scale
        # ]

        # # First test with all channels active
        # for amp_value in test_amplitudes:
        #     yield from test_with_amplitude(amp_value)
        #     for _ in range(2): yield

        # # Then test each channel individually
        # for amp_value in test_amplitudes:
        #     for channel in range(4):
        #         yield from test_with_amplitude(amp_value, active_channel=channel)
        #         for _ in range(2): yield

    run_simulation(tr, tb_generator(), vcd_name="ltc2000_datasynth_debug.vcd")

if __name__ == "__main__":
    test_bench()