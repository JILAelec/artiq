from artiq.experiment import *
from artiq.coredevice import spi2 as spi
from artiq.coredevice.core import CoreCSRBuilder

class LTC2000:
    kernel_invariants = {"core", "spi"}

    def __init__(self, dmgr, spi_device, csr_base):
        self.core = dmgr.get("core")
        self.spi = dmgr.get(spi_device)

        self.csr = CoreCSRBuilder(self.core, csr_base)
        self.ftw = self.csr.storage("ftw", 32)
        self.ptw = self.csr.storage("ptw", 32)
        self.clr = self.csr.storage("clr", 1)
        self.reset = self.csr.storage("reset", 1)

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

        # Configure FTW and PTW
        self.clr.write(1)
        ftw = self.frequency_to_ftw(frequency_mhz)
        self.ftw.write(ftw)
        self.ptw.write(0x0)
        delay(1*ms)

        # Reset the LTC2000
        self.reset.write(1)
        delay(1*ms)
        self.reset.write(0)
        delay(1*ms)

        self.clr.write(0)

    @staticmethod
    def frequency_to_ftw(desired_frequency_mhz, reference_clock_mhz=2400):
        ftw = (desired_frequency_mhz / reference_clock_mhz) * (2**32)
        return int(round(ftw))