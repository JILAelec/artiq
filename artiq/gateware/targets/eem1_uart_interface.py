from migen import *
from migen.build.generic_platform import Subsignal, Pins, IOStandard
from migen.genlib.cdc import ClockDomainsRenamer
from misoc.interconnect.stream import Endpoint, AsyncFIFO, Converter
from misoc.cores.uart import RS232PHY
from misoc.interconnect.csr import AutoCSR, CSRStatus

# EEM1 UART pin definitions (using 2 pins from EEM1)
eem1_uart_pads = [
    ("eem1_uart", 0,
        Subsignal("tx", Pins("eem1:d0_cc_p"), IOStandard("LVCMOS25")),  # Use clock-capable pin for TX
        Subsignal("rx", Pins("eem1:d0_cc_n"), IOStandard("LVCMOS25")),  # Use complementary pin for RX
    )
]

class UARTWithAsyncFIFO(Module):
    """Bidirectional UART with async FIFOs for clock domain crossing using RS232PHY"""
    def __init__(self, uart_pads, clk_freq, baudrate, fifo_depth=2048):
        # TX path: rio domain -> sys domain
        self.tx_data = Signal(8)
        self.tx_valid = Signal()
        self.tx_ready = Signal()

        # RX path: sys domain -> rio domain
        self.rx_data = Signal(8)
        self.rx_valid = Signal()
        self.rx_ready = Signal()

        # Create the RS232 PHY (runs in sys domain)
        self.submodules.phy = RS232PHY(uart_pads, clk_freq, baudrate)

        # TX Async FIFO: rio domain -> sys domain
        self.submodules.tx_fifo = ClockDomainsRenamer({
            "write": "rio",
            "read": "sys"
        })(AsyncFIFO([("data", 8)], depth=fifo_depth))

        # RX Async FIFO: sys domain -> rio domain
        self.submodules.rx_fifo = ClockDomainsRenamer({
            "write": "sys",
            "read": "rio"
        })(AsyncFIFO([("data", 8)], depth=fifo_depth))

        # Connect TX path
        self.comb += [
            # Rio domain to TX FIFO
            self.tx_fifo.sink.stb.eq(self.tx_valid),
            self.tx_fifo.sink.data.eq(self.tx_data),
            self.tx_ready.eq(self.tx_fifo.sink.ack),
        ]

        # TX FIFO to UART PHY (sys domain)
        self.sync.sys += [
            self.tx_fifo.source.connect(self.phy.sink)
        ]

        # Connect RX path
        # UART PHY to RX FIFO (sys domain)
        self.sync.sys += [
            self.phy.source.connect(self.rx_fifo.sink)
        ]

        # RX FIFO to Rio domain
        self.comb += [
            self.rx_data.eq(self.rx_fifo.source.data),
            self.rx_valid.eq(self.rx_fifo.source.stb),
            self.rx_fifo.source.ack.eq(self.rx_ready)
        ]

class CoefficientMultiplexer(Module, AutoCSR):
    """Multiplexer to route coefficient writes to multiple DDS cores and read back status"""
    def __init__(self, num_dds=4):
        # Command input interface
        self.cmd_in = Endpoint([("data", 8)])  # Command input stream

        # Direct coefficient processor connections
        self.coeff_data = [Signal(240) for _ in range(num_dds)]
        self.coeff_stb = [Signal() for _ in range(num_dds)]
        self.coeff_ack = [Signal() for _ in range(num_dds)]

        # Readback connections from coefficient processors
        self.readback_ftw = [Signal(32) for _ in range(num_dds)]
        self.readback_atw = [Signal(32) for _ in range(num_dds)]
        self.readback_ptw = [Signal(18) for _ in range(num_dds)]
        self.readback_amplitude = [Signal(16) for _ in range(num_dds)]
        self.readback_shift = [Signal(4) for _ in range(num_dds)]
        self.readback_shift_counter = [Signal(16) for _ in range(num_dds)]

        # Response output interface
        self.resp_out = Endpoint([("data", 8)])  # Response output stream

        # CSR for monitoring
        self.cmd_count = CSRStatus(32)
        self.error_count = CSRStatus(32)
        self.current_target = CSRStatus(8)
        self.last_command = CSRStatus(8)
        self.debug_state = CSRStatus(8)

        # Command definitions
        CMD_WRITE_COEFF = 0x10
        CMD_READ_COEFF = 0x20
        CMD_READ_STATUS = 0x30
        CMD_PING = 0x01

        RESP_OK = 0x00
        RESP_ERROR = 0xFF
        RESP_DATA = 0x80

        # Internal state
        cmd_counter = Signal(32)
        error_counter = Signal(32)
        target_dds = Signal(8)
        current_cmd = Signal(8)

        # Packet buffers
        cmd_buffer = Array([Signal(8) for _ in range(35)])  # cmd + addr + 30 data + 2 crc
        resp_buffer = Array([Signal(8) for _ in range(35)]) # status + 30 data + 2 crc + 2 extra
        buffer_index = Signal(6)
        resp_index = Signal(6)
        resp_length = Signal(6)

        # CRC calculation (simple XOR checksum for now)
        crc_calc = Signal(16)

        # Convert readback lists to Arrays for dynamic indexing
        readback_ftw_array = Array(self.readback_ftw)
        readback_atw_array = Array(self.readback_atw)
        readback_ptw_array = Array(self.readback_ptw)
        readback_amplitude_array = Array(self.readback_amplitude)
        readback_shift_array = Array(self.readback_shift)
        readback_shift_counter_array = Array(self.readback_shift_counter)

        # State machine
        self.submodules.fsm = fsm = FSM(reset_state="WAIT_SYNC")

        # Sync pattern detection
        sync_buffer = Array([Signal(8) for _ in range(3)])

        fsm.act("WAIT_SYNC",
            self.cmd_in.ack.eq(1),
            self.debug_state.status.eq(0),
            # Clear all strobes
            [self.coeff_stb[i].eq(0) for i in range(num_dds)],
            If(self.cmd_in.stb,
                NextValue(sync_buffer[0], sync_buffer[1]),
                NextValue(sync_buffer[1], sync_buffer[2]),
                NextValue(sync_buffer[2], self.cmd_in.payload.data),
                If((sync_buffer[0] == 0xAA) &
                   (sync_buffer[1] == 0x55) &
                   (sync_buffer[2] == 0xCC),  # Different sync for commands
                    NextValue(buffer_index, 0),
                    NextValue(crc_calc, 0),
                    NextState("RECEIVE_COMMAND")
                )
            )
        )

        fsm.act("RECEIVE_COMMAND",
            self.cmd_in.ack.eq(1),
            self.debug_state.status.eq(1),
            # Clear all strobes
            [self.coeff_stb[i].eq(0) for i in range(num_dds)],
            If(self.cmd_in.stb,
                NextValue(cmd_buffer[buffer_index], self.cmd_in.payload.data),
                NextValue(crc_calc, crc_calc ^ self.cmd_in.payload.data),
                NextValue(buffer_index, buffer_index + 1),
                If(buffer_index == 0,  # Command byte
                    NextValue(current_cmd, self.cmd_in.payload.data)
                ).Elif(buffer_index == 1,  # Address byte
                    NextValue(target_dds, self.cmd_in.payload.data)
                ).Elif(buffer_index == 32,  # Got command + address + 30 data bytes + 1 CRC byte
                    NextState("PROCESS_COMMAND")
                )
            )
        )

        fsm.act("PROCESS_COMMAND",
            self.debug_state.status.eq(2),
            # Clear all strobes
            [self.coeff_stb[i].eq(0) for i in range(num_dds)],
            # Validate CRC (simple check for now)
            If((cmd_buffer[32] == (crc_calc & 0xFF)) & (target_dds < num_dds),
                NextValue(cmd_counter, cmd_counter + 1),
                Case(current_cmd, {
                    CMD_WRITE_COEFF: NextState("WRITE_COEFFICIENT"),
                    CMD_READ_COEFF: NextState("READ_COEFFICIENT"),
                    CMD_READ_STATUS: NextState("READ_STATUS"),
                    CMD_PING: NextState("SEND_PING_RESPONSE"),
                    "default": [
                        NextValue(error_counter, error_counter + 1),
                        NextState("SEND_ERROR_RESPONSE")
                    ]
                })
            ).Else(
                NextValue(error_counter, error_counter + 1),
                NextState("SEND_ERROR_RESPONSE")
            )
        )

        fsm.act("WRITE_COEFFICIENT",
            self.debug_state.status.eq(3),
            # Assert strobe for target DDS
            If(target_dds < num_dds,
                Case(target_dds, {
                    i: self.coeff_stb[i].eq(1) for i in range(num_dds)
                }),
                # Wait for acknowledge (simple 1-cycle strobe for now)
                NextState("SEND_OK_RESPONSE")
            ).Else(
                NextState("SEND_ERROR_RESPONSE")
            )
        )

        fsm.act("READ_COEFFICIENT",
            self.debug_state.status.eq(4),
            # Clear all strobes
            [self.coeff_stb[i].eq(0) for i in range(num_dds)],
            # Return actual coefficient data from target DDS
            NextValue(resp_buffer[0], RESP_DATA),
            If(target_dds < num_dds,
                # Pack current coefficient data in same format as write command
                # This reconstructs the 240-bit coefficient format from individual signals
                NextValue(resp_buffer[1], readback_amplitude_array[target_dds] & 0xFF),          # amp[7:0]
                NextValue(resp_buffer[2], (readback_amplitude_array[target_dds] >> 8) & 0xFF),   # amp[15:8]
                NextValue(resp_buffer[3], readback_atw_array[target_dds] & 0xFF),                # atw[7:0]
                NextValue(resp_buffer[4], (readback_atw_array[target_dds] >> 8) & 0xFF),         # atw[15:8]
                NextValue(resp_buffer[5], (readback_atw_array[target_dds] >> 16) & 0xFF),        # atw[23:16]
                NextValue(resp_buffer[6], (readback_atw_array[target_dds] >> 24) & 0xFF),        # atw[31:24]
                NextValue(resp_buffer[7], readback_ftw_array[target_dds] & 0xFF),                # ftw[7:0]
                NextValue(resp_buffer[8], (readback_ftw_array[target_dds] >> 8) & 0xFF),         # ftw[15:8]
                NextValue(resp_buffer[9], (readback_ftw_array[target_dds] >> 16) & 0xFF),        # ftw[23:16]
                NextValue(resp_buffer[10], (readback_ftw_array[target_dds] >> 24) & 0xFF),       # ftw[31:24]
                NextValue(resp_buffer[11], readback_ptw_array[target_dds] & 0xFF),               # ptw[7:0]
                NextValue(resp_buffer[12], (readback_ptw_array[target_dds] >> 8) & 0xFF),        # ptw[15:8]
                NextValue(resp_buffer[13], (readback_ptw_array[target_dds] >> 16) & 0x03),       # ptw[17:16]
                NextValue(resp_buffer[14], readback_shift_array[target_dds] & 0x0F),             # shift[3:0]
                NextValue(resp_buffer[15], readback_shift_counter_array[target_dds] & 0xFF),     # shift_counter[7:0]
                NextValue(resp_buffer[16], (readback_shift_counter_array[target_dds] >> 8) & 0xFF), # shift_counter[15:8]
                # Fill remaining bytes with zeros
                [NextValue(resp_buffer[i+17], 0) for i in range(14)],
                NextValue(resp_length, 32)
            ).Else(
                # Invalid target - return error data
                [NextValue(resp_buffer[i+1], 0xFF) for i in range(30)],
                NextValue(resp_length, 32)
            ),
            NextValue(resp_buffer[31], 0),  # CRC placeholder
            NextValue(resp_index, 0),
            NextState("SEND_RESPONSE")
        )

        fsm.act("READ_STATUS",
            self.debug_state.status.eq(5),
            # Clear all strobes
            [self.coeff_stb[i].eq(0) for i in range(num_dds)],
            # Return current status of target DDS with real data
            NextValue(resp_buffer[0], RESP_DATA),
            If(target_dds < num_dds,
                NextValue(resp_buffer[1], target_dds),                                          # Echo target
                NextValue(resp_buffer[2], current_cmd),                                         # Echo command
                NextValue(resp_buffer[3], readback_ftw_array[target_dds] & 0xFF),               # FTW[7:0]
                NextValue(resp_buffer[4], (readback_ftw_array[target_dds] >> 8) & 0xFF),        # FTW[15:8]
                NextValue(resp_buffer[5], (readback_ftw_array[target_dds] >> 16) & 0xFF),       # FTW[23:16]
                NextValue(resp_buffer[6], (readback_ftw_array[target_dds] >> 24) & 0xFF),       # FTW[31:24]
                NextValue(resp_buffer[7], readback_amplitude_array[target_dds] & 0xFF),         # Amplitude[7:0]
                NextValue(resp_buffer[8], (readback_amplitude_array[target_dds] >> 8) & 0xFF),  # Amplitude[15:8]
                NextValue(resp_buffer[9], readback_atw_array[target_dds] & 0xFF),               # ATW[7:0]
                NextValue(resp_buffer[10], (readback_atw_array[target_dds] >> 8) & 0xFF),       # ATW[15:8]
                NextValue(resp_buffer[11], (readback_atw_array[target_dds] >> 16) & 0xFF),      # ATW[23:16]
                NextValue(resp_buffer[12], (readback_atw_array[target_dds] >> 24) & 0xFF),      # ATW[31:24]
                NextValue(resp_buffer[13], readback_ptw_array[target_dds] & 0xFF),              # PTW[7:0]
                NextValue(resp_buffer[14], (readback_ptw_array[target_dds] >> 8) & 0xFF),       # PTW[15:8]
                NextValue(resp_buffer[15], (readback_ptw_array[target_dds] >> 16) & 0x03),      # PTW[17:16]
                NextValue(resp_length, 16)
            ).Else(
                # Invalid target
                [NextValue(resp_buffer[i+1], 0xFF) for i in range(14)],
                NextValue(resp_length, 16)
            ),
            NextValue(resp_buffer[15], 0),  # CRC placeholder
            NextValue(resp_index, 0),
            NextState("SEND_RESPONSE")
        )

        fsm.act("SEND_PING_RESPONSE",
            self.debug_state.status.eq(6),
            # Clear all strobes
            [self.coeff_stb[i].eq(0) for i in range(num_dds)],
            NextValue(resp_buffer[0], RESP_OK),
            NextValue(resp_buffer[1], 0x42),  # Ping response signature
            NextValue(resp_buffer[2], target_dds),
            NextValue(resp_buffer[3], 0),  # CRC placeholder
            NextValue(resp_length, 4),
            NextValue(resp_index, 0),
            NextState("SEND_RESPONSE")
        )

        fsm.act("SEND_OK_RESPONSE",
            self.debug_state.status.eq(7),
            # Clear all strobes
            [self.coeff_stb[i].eq(0) for i in range(num_dds)],
            NextValue(resp_buffer[0], RESP_OK),
            NextValue(resp_buffer[1], target_dds),
            NextValue(resp_buffer[2], current_cmd),
            NextValue(resp_buffer[3], 0),  # CRC placeholder
            NextValue(resp_length, 4),
            NextValue(resp_index, 0),
            NextState("SEND_RESPONSE")
        )

        fsm.act("SEND_ERROR_RESPONSE",
            self.debug_state.status.eq(8),
            # Clear all strobes
            [self.coeff_stb[i].eq(0) for i in range(num_dds)],
            NextValue(resp_buffer[0], RESP_ERROR),
            NextValue(resp_buffer[1], target_dds),
            NextValue(resp_buffer[2], current_cmd),
            NextValue(resp_buffer[3], 0),  # CRC placeholder
            NextValue(resp_length, 4),
            NextValue(resp_index, 0),
            NextState("SEND_RESPONSE")
        )

        fsm.act("SEND_RESPONSE",
            self.debug_state.status.eq(9),
            # Clear all strobes
            [self.coeff_stb[i].eq(0) for i in range(num_dds)],
            self.resp_out.stb.eq(1),
            self.resp_out.payload.data.eq(resp_buffer[resp_index]),
            If(self.resp_out.ack,
                NextValue(resp_index, resp_index + 1),
                If(resp_index == resp_length - 1,
                    NextState("WAIT_SYNC")
                )
            )
        )

        # Connect coefficient data outputs
        for i in range(num_dds):
            self.comb += [
                self.coeff_data[i].eq(
                    Cat(*[cmd_buffer[j+2] for j in range(30)])  # Skip cmd and addr bytes
                )
            ]

        # Update CSR status
        self.comb += [
            self.cmd_count.status.eq(cmd_counter),
            self.error_count.status.eq(error_counter),
            self.current_target.status.eq(target_dds),
            self.last_command.status.eq(current_cmd)
        ]

class ResponseFormatter(Module):
    """Formats responses and sends sync pattern + data"""
    def __init__(self):
        # Input from multiplexer
        self.data_in = Endpoint([("data", 8)])

        # Output to UART TX
        self.uart_out = Signal(8)
        self.uart_valid = Signal()
        self.uart_ready = Signal()

        # State machine to add sync pattern
        sync_sent = Signal(2)

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")

        fsm.act("IDLE",
            self.data_in.ack.eq(0),
            If(self.data_in.stb,
                NextValue(sync_sent, 0),
                NextState("SEND_SYNC")
            )
        )

        fsm.act("SEND_SYNC",
            self.uart_valid.eq(1),
            If(self.uart_ready,
                NextValue(sync_sent, sync_sent + 1),
                If(sync_sent == 2,
                    NextState("SEND_DATA")
                )
            )
        )

        fsm.act("SEND_DATA",
            self.uart_valid.eq(1),
            self.uart_out.eq(self.data_in.payload.data),
            self.data_in.ack.eq(self.uart_ready),
            If(self.uart_ready & ~self.data_in.stb,
                # End of data
                NextState("IDLE")
            )
        )

        # Sync pattern output
        self.comb += [
            If(fsm.ongoing("SEND_SYNC"),
                Case(sync_sent, {
                    0: self.uart_out.eq(0xAA),
                    1: self.uart_out.eq(0x55),
                    2: self.uart_out.eq(0xDD)  # Different sync for responses
                })
            )
        ]

class EEM1UARTCoefficientInterface(Module, AutoCSR):
    """Complete bidirectional UART interface for multiple coefficient processors"""
    def __init__(self, platform, sys_clk_freq, num_dds=4, baud_rate=115200):
        # Add EEM1 UART pins to platform
        platform.add_extension(eem1_uart_pads)
        uart_pads = platform.request("eem1_uart", 0)

        # Bidirectional UART with async FIFOs using RS232PHY
        # Run the whole UART module without clock domain renaming first
        self.submodules.uart_fifo = UARTWithAsyncFIFO(uart_pads, sys_clk_freq, baud_rate)

        # Command multiplexer (runs in rio domain)
        self.submodules.cmd_mux = ClockDomainsRenamer("rio")(
            CoefficientMultiplexer(num_dds))

        # Response formatter (runs in rio domain)
        self.submodules.resp_formatter = ClockDomainsRenamer("rio")(
            ResponseFormatter())

        # Connect UART RX to command multiplexer
        self.comb += [
            self.cmd_mux.cmd_in.payload.data.eq(self.uart_fifo.rx_data),
            self.cmd_mux.cmd_in.stb.eq(self.uart_fifo.rx_valid),
            self.uart_fifo.rx_ready.eq(self.cmd_mux.cmd_in.ack)
        ]

        # Connect multiplexer response to formatter
        self.comb += [
            self.resp_formatter.data_in.connect(self.cmd_mux.resp_out)
        ]

        # Connect formatter to UART TX
        self.comb += [
            self.uart_fifo.tx_data.eq(self.resp_formatter.uart_out),
            self.uart_fifo.tx_valid.eq(self.resp_formatter.uart_valid),
            self.resp_formatter.uart_ready.eq(self.uart_fifo.tx_ready)
        ]

        # Direct outputs for connecting to coefficient processors (WRITE)
        self.coeff_data = self.cmd_mux.coeff_data    # [Signal(240)] * num_dds
        self.coeff_stb = self.cmd_mux.coeff_stb      # [Signal()] * num_dds
        self.coeff_ack = self.cmd_mux.coeff_ack      # [Signal()] * num_dds

        # Direct inputs for reading from coefficient processors (READ)
        self.readback_ftw = self.cmd_mux.readback_ftw                    # [Signal(32)] * num_dds
        self.readback_atw = self.cmd_mux.readback_atw                    # [Signal(32)] * num_dds
        self.readback_ptw = self.cmd_mux.readback_ptw                    # [Signal(18)] * num_dds
        self.readback_amplitude = self.cmd_mux.readback_amplitude        # [Signal(16)] * num_dds
        self.readback_shift = self.cmd_mux.readback_shift                # [Signal(4)] * num_dds
        self.readback_shift_counter = self.cmd_mux.readback_shift_counter # [Signal(16)] * num_dds

        # CSR interfaces for debugging
        self.csr_devices = ["cmd_mux"]