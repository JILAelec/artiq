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

    def tb_generator():
        LATENCY = 2  # Pipeline latency in cycles
        amp_value = 0x4000  # 1/4 scale

        # Set amplitudes
        for dds in range(4):
            for phase in range(24):
                yield dut.amplitudes[dds][phase].eq(amp_value)
        yield

        # Store input history for comparison
        input_history = []

        # Run test cycles
        for cycle in range(10):
            if cycle < len(sample_data):
                input_value = sample_data[cycle]
                input_history.append(input_value)
                print(f"\nCycle {cycle}: Setting input value {input_value:04x}")

                # Set inputs
                for dds in range(4):
                    for phase in range(24):
                        yield dut.data_in[dds][phase].eq(input_value)

            # Clock the circuit
            yield

            # Monitor outputs
            if cycle >= LATENCY:
                output = (yield dut.summers[0].output)
                expected_input = input_history[cycle-LATENCY]

                print(f"Phase 0 Output: {output & 0xFFFF:04x} (from input {expected_input:04x} at cycle {cycle-LATENCY})")

                # Verify amplitude scaling
                for dds in range(4):
                    input_val = (yield dut.summers[0].inputs[dds])
                    amp_val = (yield dut.summers[0].amplitudes[dds])
                    if input_val != input_history[-1]:
                        print(f"  WARNING: DDS {dds} input mismatch: got {input_val:04x}, expected {input_history[-1]:04x}")
                    if amp_val != amp_value:
                        print(f"  WARNING: DDS {dds} amplitude mismatch: got {amp_val:04x}, expected {amp_value:04x}")

    run_simulation(dut, tb_generator(), vcd_name="ltc2000_datasynth_debug.vcd")

if __name__ == "__main__":
    test_bench()