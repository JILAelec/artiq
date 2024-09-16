from artiq.experiment import *
from artiq.coredevice import spi2 as spi

class LTC2000Driver:
    kernel_invariants = {"core", "spi", "channel"}

    def __init__(self, dmgr, spi_device, channel):
        self.core = dmgr.get("core")
        self.spi = dmgr.get(spi_device)
        self.channel = channel

        self.spi_config = (0*spi.SPI_OFFLINE | 0*spi.SPI_END |
                           0*spi.SPI_INPUT | 0*spi.SPI_CS_POLARITY |
                           0*spi.SPI_CLK_POLARITY | 0*spi.SPI_CLK_PHASE |
                           0*spi.SPI_LSB_FIRST | 0*spi.SPI_HALF_DUPLEX)
        self.spi_div = 32
        self.spi_cs = 0

    @staticmethod
    def create_command(address, data, is_read):
        return (int(is_read) << 15) | (address << 8) | data

    @kernel
    def write_reg(self, address, data):
        command = self.create_command(address, data, False)
        self.spi.write(command)

    @kernel
    def read_reg(self, address):
        command = self.create_command(address, 0, True)
        return self.spi.write(command) & 0xFF

    @kernel
    def initialize(self):
        self.spi.set_config_mu(self.spi_config, 16, self.spi_div, self.spi_cs)

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
            self.write_reg(address, data)
            delay(1*ms)  # Add a small delay between operations

    @kernel
    def configure(self, frequency_mhz=200.0):
        # Configure DCKI controls
        self.write_reg(0x03, 0x01)
        delay(1*ms)

        # Configure data input controls
        self.write_reg(0x04, 0x0B)
        delay(1*ms)

        # Configure FTW
        ftw = self.frequency_to_ftw(frequency_mhz)
        self.set_ftw(ftw)
        delay(1*ms)

        # Reset the LTC2000
        self.reset()

    @kernel
    def set_ftw(self, ftw):
        # Assuming FTW is sent as two 16-bit words
        self.write_reg(0x0E, (ftw >> 16) & 0xFFFF)  # Upper 16 bits
        self.write_reg(0x0F, ftw & 0xFFFF)          # Lower 16 bits

    @kernel
    def reset(self):
        self.write_reg(0x01, 0x01)  # Set reset bit
        delay(1*us)
        self.write_reg(0x01, 0x00)  # Clear reset bit
        delay(1*ms)

    @portable
    def frequency_to_ftw(self, desired_frequency_mhz, reference_clock_mhz=2400):
        ftw = (desired_frequency_mhz / reference_clock_mhz) * (2**32)
        return int(round(ftw))