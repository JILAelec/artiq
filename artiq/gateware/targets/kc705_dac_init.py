from artiq.experiment import *
from artiq.coredevice.spi2 import SPI_INPUT, SPI_END, SPI_CLK_POLARITY, SPI_CLK_PHASE
from artiq.language.units import us, ms

class DAC_Init(EnvExperiment):
    def build(self):
        self.setattr_device("core")
        self.setattr_device("ltc2000")
        self.setattr_device("spi_ltc")
        self.spi_config = 0  # Initialize spi_config attribute

    @kernel
    def run(self):
        self.core.reset()
        spi_modes = [(0,0)]
        for cpol, cpha in spi_modes:
            print("Testing SPI Mode:", cpol, cpha)
            self.init_spi(cpol, cpha)
            self.test_spi_operations()
            print("Reading all registers...")
            self.read_all_registers()

    @kernel
    def init_spi(self, cpol, cpha):
        self.core.break_realtime()
        self.spi_config = SPI_END
        if cpol:
            self.spi_config |= SPI_CLK_POLARITY
        if cpha:
            self.spi_config |= SPI_CLK_PHASE
        self.spi_ltc.set_config_mu(self.spi_config, 32, 256, 1)  # Slow clock for stability
        print("SPI Config:", self.spi_config)

    @kernel
    def spi_write(self, addr, data):
        self.core.break_realtime()
        self.spi_ltc.set_config_mu(self.spi_config, 32, 256, 0b0001)
        self.spi_ltc.write((addr << 24) | (data << 16))
        delay(5*us)
        self.spi_ltc.set_config_mu(self.spi_config, 32, 256, 0b0000)
        print("SPI Write - Addr:", addr, "Data:", data)

    @kernel
    def spi_read(self, addr):
        self.core.break_realtime()
        self.spi_ltc.set_config_mu(self.spi_config | SPI_INPUT, 32, 256, 0b0001)
        self.spi_ltc.write((1 << 31) | (addr << 24))
        delay(5*us)
        result = self.spi_ltc.read()
        self.spi_ltc.set_config_mu(self.spi_config, 32, 256, 0b0000)
        byte_3 = (result >> 24) & 0xFF
        byte_2 = (result >> 16) & 0xFF
        byte_1 = (result >> 8) & 0xFF
        byte_0 = result & 0xFF
        print("SPI Read - Addr:", addr, "Full Result:", result, "Bytes:", byte_3, byte_2, byte_1, byte_0)
        return byte_2  # Return the second most significant byte

    @kernel
    def test_spi_operations(self):
        test_registers = [0x02, 0x04, 0x1E]
        test_values = [0x30, 0x0F, 0x01]
        for i in range(len(test_registers)):
            addr = test_registers[i]
            value = test_values[i]
            old_value = self.spi_read(addr)
            print("Register", addr, "before write:", old_value)
            self.spi_write(addr, value)
            delay(1*ms)
            new_value = self.spi_read(addr)
            print("Wrote", value, "to register", addr, ", Read back:", new_value)
            if int(new_value) != value and addr != 0x02:  # Exclude register 2 which is likely read-only
                print("Warning: Write to register", addr, "failed!")

    @kernel
    def read_all_registers(self):
        for addr in range(32):
            value = self.spi_read(addr)
            print("Register", addr, ":", value)