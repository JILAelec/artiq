from artiq.experiment import *
from artiq.coredevice import spi2 as spi

LTC_SPI_CONFIG = (0*spi.SPI_OFFLINE | 0*spi.SPI_END |
                  0*spi.SPI_INPUT | 0*spi.SPI_CS_POLARITY |
                  0*spi.SPI_CLK_POLARITY | 0*spi.SPI_CLK_PHASE |
                  0*spi.SPI_LSB_FIRST | 0*spi.SPI_HALF_DUPLEX)
LTC_SPI_DIV = 32
LTC_CS = 0

class LTC2000Experiment(EnvExperiment):
    def build(self):
        self.setattr_device("core")
        self.setattr_device("spi_ltc")
        self.setattr_device("ttl1")
        self.register_values = {}

    def create_command(self, address, data, is_read):
        return (int(is_read) << 15) | (address << 8) | data

    @kernel
    def write_ltc2000(self, address, data):
        command = self.create_command(address, data, False)
        self.spi_ltc.write(command)

    @kernel
    def read_ltc2000(self, address):
        command = self.create_command(address, 0, True)
        return self.spi_ltc.write(command)

    @kernel
    def initialize_ltc2000(self):
        self.spi_ltc.set_config_mu(LTC_SPI_CONFIG, 16, LTC_SPI_DIV, LTC_CS)

        registers = [
            (0x01, 0x00),  # Reset, power down controls
            (0x02, 0x00),  # Clock and DCKO controls
            (0x03, 0x00),  # DCKI controls
            (0x04, 0x00),  # Data input controls
            (0x05, 0x00),  # Synchronizer controls
            (0x06, 0x00),  # Synchronizer phase (read-only)
            (0x07, 0x00),  # Linearization controls
            (0x08, 0x08),  # Linearization voltage controls
            (0x09, 0x00),  # Gain adjustment
            (0x18, 0x00),  # LVDS test MUX controls
            (0x19, 0x00),  # Temperature measurement controls
            (0x1E, 0x00),  # Pattern generator enable
            (0x1F, 0x00)   # Pattern generator data
        ]

        for address, data in registers:
            self.write_ltc2000(address, data)
            read_value = self.read_ltc2000(address) & 0xFF
            self.register_values[address] = read_value
            delay(1*ms)  # Add a small delay between operations

    @kernel
    def configure_ltc2000(self):
        # Write 0x01 to address 0x03
        self.write_ltc2000(0x03, 0x01)
        self.register_values[0x03] = self.read_ltc2000(0x03) & 0xFF
        delay(1*ms)

        # Write 0x0B to address 0x04
        self.write_ltc2000(0x04, 0x0B)
        self.register_values[0x04] = self.read_ltc2000(0x04) & 0xFF
        delay(1*ms)

    @kernel
    def run(self):
        self.core.reset()
        self.ttl1.on()

        self.initialize_ltc2000()
        self.configure_ltc2000()

        for i in range(10):
            self.spi_ltc.write(0xFFFF)
            delay(50*ms)

        self.ttl1.off()

    def analyze(self):
        print("LTC2000 Register Values:")
        for address, value in sorted(self.register_values.items()):
            print(f"Register 0x{address:02X}: 0x{value:02X}")

        # Check for specific configurations
        if self.register_values.get(0x03) == 0x01:
            print("DCKIP/N clock receiver is enabled.")
        else:
            print("DCKIP/N clock receiver is not enabled as expected.")

        if self.register_values.get(0x04) == 0x0B:
            print("Port A and B LVDS receivers and DAC data are enabled.")
        else:
            print("Port A and B LVDS receivers and DAC data are not configured as expected.")