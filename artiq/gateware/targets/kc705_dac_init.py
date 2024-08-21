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

        spi_modes = [(0,0), (0,1), (1,0), (1,1)]
        #let's do only one mode for now
        spi_modes = [(0,0)]
        for cpol, cpha in spi_modes:
            print("Testing SPI Mode:", cpol, cpha)
            self.init_spi(cpol, cpha)
            self.test_spi_write()
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
        # Set CS line low (assert CS)
        self.spi_ltc.set_config_mu(self.spi_config | SPI_END, 32, 256, 0b0001)
        self.spi_ltc.write((addr << 24) | (data << 16))
        delay(200*us)  # Increased delay for stability
        # Set CS line high (deassert CS)
        self.spi_ltc.set_config_mu(self.spi_config, 32, 256, 0b0000)
        delay(2*ms)  # Increased delay for stability
        print("SPI Write - Addr:", addr, "Data:", data)

    @kernel
    def spi_read(self, addr):
        self.core.break_realtime()
        # Set CS line low (assert CS)
        self.spi_ltc.set_config_mu(self.spi_config | SPI_END | SPI_INPUT, 32, 256, 0b0001)
        self.spi_ltc.write((1 << 31) | (addr << 24))
        delay(200*us)  # Increased delay for stability
        result = self.spi_ltc.read() & 0xFF
        # Set CS line high (deassert CS)
        self.spi_ltc.set_config_mu(self.spi_config, 32, 256, 0b0000)
        delay(2*ms)  # Increased delay for stability
        print("SPI Read - Addr:", addr, "Result:", result)
        return result

    @kernel
    def test_spi_write(self):
        test_registers = [0x02, 0x04, 0x1E]
        test_values = [0x30, 0x0F, 0x01]
        for i in range(len(test_registers)):
            addr = test_registers[i]
            value = test_values[i]
            old_value = self.spi_read(addr)
            print("Register", addr, "before write:", old_value)
            self.spi_write(addr, value)
            new_value = self.spi_read(addr)
            print("Wrote", value, "to register", addr, ", Read back:", new_value)

    @kernel
    def read_all_registers(self):
        for addr in range(32):
            value = self.spi_read(addr)
            print("Register", addr, ":", value)