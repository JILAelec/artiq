from artiq.experiment import *
from artiq.coredevice import spi2

class LTC2000:
    kernel_invariants = {"core", "spi"}

    def __init__(self, dmgr, spi_device):
        self.core = dmgr.get_device("core")
        self.spi = dmgr.get_device(spi_device)

    @kernel
    def init(self):
        self.core.break_realtime()
        # Combine configuration flags into a single value
        config = (0 * spi2.SPI_OFFLINE |
                  0 * spi2.SPI_END |
                  0 * spi2.SPI_INPUT |
                  1 * spi2.SPI_CS_POLARITY |  # Active-low chip select
                  0 * spi2.SPI_CLK_POLARITY |
                  0 * spi2.SPI_CLK_PHASE |
                  0 * spi2.SPI_LSB_FIRST |
                  0 * spi2.SPI_HALF_DUPLEX)
        self.spi.set_config_mu(config, 16, 4, 0)

    @kernel
    def write(self, addr, data):
        self.spi.write(((addr & 0x7F) << 8) | (data & 0xFF))

    @kernel
    def read(self, addr):
        return self.spi.write((1 << 15) | ((addr & 0x7F) << 8)) & 0xFF

    @kernel
    def set_frequency(self, freq):
        ftw = self.frequency_to_ftw(freq)
        self.write(0x0E, (ftw >> 16) & 0xFFFF)
        self.write(0x0F, ftw & 0xFFFF)

    @portable
    def frequency_to_ftw(self, freq):
        return round((freq / 2400e6) * (1 << 32))

    @kernel
    def set_amplitude(self, amplitude):
        amp = round(amplitude * 0x3FFF)
        self.write(0x09, amp & 0xFFFF)

    @kernel
    def set_phase(self, phase):
        phase_word = round((phase % 360) / 360 * 0xFFFF)
        self.write(0x10, phase_word & 0xFFFF)

    @kernel
    def power_down(self):
        self.write(0x01, 0x10)

    @kernel
    def power_up(self):
        self.write(0x01, 0x00)

    @kernel
    def initialize(self):
        self.init()
        self.write(0x01, 0x00)  # Reset, power down controls
        self.write(0x02, 0x00)  # Clock and DCKO controls
        self.write(0x03, 0x01)  # DCKI controls
        self.write(0x04, 0x0B)  # Data input controls
        self.write(0x05, 0x00)  # Synchronizer controls
        self.write(0x07, 0x00)  # Linearization controls
        self.write(0x08, 0x08)  # Linearization voltage controls
        self.write(0x18, 0x00)  # LVDS test MUX controls
        self.write(0x19, 0x00)  # Temperature measurement controls
        self.write(0x1E, 0x00)  # Pattern generator enable
        self.write(0x1F, 0x00)  # Pattern generator data

    @kernel
    def configure(self, frequency, amplitude, phase):
        self.set_frequency(frequency)
        self.set_amplitude(amplitude)
        self.set_phase(phase)
        self.power_up()