from migen import *
from migen.fhdl import verilog
from migen.sim import run_simulation
from ltc2000 import LTC2000DataSynth

def test_bench():
    dut = LTC2000DataSynth(NUM_OF_DDS=4, NPHASES=24)

    sample_data = [
        0x0000, 0x2120, 0x3fff, 0x5a81, 0x6ed9, 0x7ba2, 0x7fff, 0x7ba2, 0x6ed9, 0x5a81,
        0x3fff, 0x2120, 0x0000, 0xdee0, 0xc001, 0xa57f, 0x9127, 0x845e, 0x8001, 0x845e,
        0x9127, 0xa57f, 0xc001, 0xdee0
    ]

    def test_with_amplitude(amp_value, active_channel=None):
        """
        Run test with given amplitude. If active_channel is specified (0-3),
        only that channel will receive input while others are set to 0.
        """
        LATENCY = 2
        test_type = "single channel" if active_channel is not None else "all channels"
        print(f"\n=== Testing with amplitude {amp_value:04x} ({test_type}) ===")

        # Set amplitudes
        for dds in range(4):
            for phase in range(24):
                yield dut.amplitudes[dds][phase].eq(amp_value)
        yield

        input_history = []

        for cycle in range(10):
            if cycle < len(sample_data):
                input_value = sample_data[cycle]
                input_history.append(input_value)
                print(f"\nCycle {cycle}: Setting input value {input_value:04x}")

                # Set inputs based on test mode
                for dds in range(4):
                    for phase in range(24):
                        if active_channel is None or dds == active_channel:
                            yield dut.data_in[dds][phase].eq(input_value)
                        else:
                            yield dut.data_in[dds][phase].eq(0)

            yield

            if cycle >= LATENCY:
                output = (yield dut.summers[0].output)
                expected_input = input_history[cycle-LATENCY]

                # Calculate expected output based on test mode
                multiplier = 4 if active_channel is None else 1
                product = (expected_input * amp_value * multiplier)
                expected_output = product >> 16

                # Apply saturation
                if expected_output > 32767:
                    expected_output = 32767
                elif expected_output < -32768:
                    expected_output = -32768
                expected_output = expected_output & 0xFFFF

                print(f"Phase 0 Output: {output & 0xFFFF:04x} (from input {expected_input:04x})")
                print(f"Expected Output: {expected_output:04x} with scaling {amp_value:04x}")

                if abs((output & 0xFFFF) - expected_output) > 2:
                    print(f"WARNING: Output differs from expected!")

    def tb_generator():
        test_amplitudes = [
            0x2000,  # 1/8 scale
            0x4000,  # 1/4 scale
            0x8000,  # 1/2 scale
            0xFFFF   # Full scale
        ]

        # First test with all channels active
        for amp_value in test_amplitudes:
            yield from test_with_amplitude(amp_value)
            for _ in range(2): yield

        # Then test each channel individually
        for amp_value in test_amplitudes:
            for channel in range(4):
                yield from test_with_amplitude(amp_value, active_channel=channel)
                for _ in range(2): yield

    run_simulation(dut, tb_generator(), vcd_name="ltc2000_datasynth_debug.vcd")

if __name__ == "__main__":
    test_bench()