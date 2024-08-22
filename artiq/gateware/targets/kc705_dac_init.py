from artiq.experiment import *
from artiq.coredevice.spi2 import SPI_INPUT, SPI_END
from artiq.language.units import us, ms

class DAC_Init(EnvExperiment):
    def build(self):
        self.setattr_device("core")
        self.setattr_device("ttl0")
        self.setattr_device("ltc2000")
        self.setattr_device("spi_ltc")
        self.spi_config = SPI_END

    @kernel
    def run(self):
        self.core.reset()
        self.ttl0.output()
        print("Performing software reset...")
        self.spi_write(0x00, 0x01)  # Write 1 to the reset bit
        delay(10*ms)  # Wait for reset to complete
        self.spi_write(0x00, 0x00)  # Clear the reset bit
        delay(10*ms)  # Wait after reset
        print("Initializing the LTC2000...")
        self.initialize()
        print("Verifying initialization...")
        self.verify_initialization()

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
        value = (result >> 16) & 0xFF  # Extract the second most significant byte
        print("SPI Read - Addr:", addr, "Value:", value)
        return value

    @kernel
    def initialize(self):
        self.spi_write(0x01, 0x00)  # Reset, power down controls
        self.spi_write(0x02, 0x00)  # Clock and DCKO controls
        self.spi_write(0x03, 0x01)  # DCKI controls
        self.spi_write(0x04, 0x0B)  # Data input controls
        self.spi_write(0x05, 0x00)  # Synchronizer controls
        self.spi_write(0x07, 0x00)  # Linearization controls
        self.spi_write(0x08, 0x08)  # Linearization voltage controls
        self.spi_write(0x18, 0x00)  # LVDS test MUX controls
        self.spi_write(0x19, 0x00)  # Temperature measurement controls
        self.spi_write(0x1E, 0x00)  # Pattern generator enable
        self.spi_write(0x1F, 0x00)  # Pattern generator data

    @kernel
    def verify_initialization(self):
        register_addresses = [1, 2, 3, 4, 5, 7, 8, 24, 25, 30]
        expected_values = [0, 0, 1, 11, 0, 0, 8, 0, 0, 0]
        for i in range(len(register_addresses)):
            addr = register_addresses[i]
            expected = expected_values[i]
            read_value = self.spi_read(addr)
            print("Register")
            print(addr)
            print("expected")
            print(expected)
            print("read")
            print(read_value)
            if self.compare_values(read_value, expected) != 0:
                print("Warning: Register mismatch")
                print(addr)

    @kernel
    def compare_values(self, a: TInt32, b: TInt32) -> TInt32:
        if a < b:
            return -1
        elif a > b:
            return 1
        return 0