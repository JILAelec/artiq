from artiq.coredevice.rtio import rtio_output
from artiq.experiment import *
from artiq.coredevice import spi2
from artiq.language.core import kernel, delay
from artiq.language.units import us

class LTC2000:
    def __init__(self, dmgr, channel, spi_device):
        self.spi = dmgr.get(spi_device)
        self.bus_channel = channel
        self.ftw_per_hz = (2**32) / 2400e6
        # channels for rtlink interface
        self.data_channel = channel      # Parameter data writes
        self.trigger_channel = channel+1 # Parameter trigger

    @kernel
    def init(self):
        """Initialize SPI interface"""
        config = (0 * spi2.SPI_OFFLINE |
                 1 * spi2.SPI_END |
                 0 * spi2.SPI_INPUT |
                 0 * spi2.SPI_CS_POLARITY |
                 0 * spi2.SPI_CLK_POLARITY |
                 0 * spi2.SPI_CLK_PHASE |
                 0 * spi2.SPI_LSB_FIRST |
                 0 * spi2.SPI_HALF_DUPLEX)
        self.spi.set_config_mu(config, 16, 32, 1)

    @kernel
    def write(self, addr, data):
        """Write to LTC2000 registers via SPI"""
        self.spi.write(((addr & 0x7F) << 24) | ((data & 0xFF) << 16))

    @kernel
    def read(self, addr):
        """Read LTC2000 registers via SPI"""
        return self.spi.write((1 << 31) | ((addr & 0x7F) << 24)) & 0xFF000000

    @kernel
    def initialize(self):
        """Full initialization sequence for LTC2000"""
        self.init()
        # LTC2000 register configuration
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

    # Parameter interface methods
    @kernel
    def write_param_chunk(self, addr, data):
        """Write 16-bit chunk of parameter data"""
        rtio_output(self.data_channel, addr, data & 0xFFFF)
        delay(1*us)

    @kernel
    def trigger(self):
        """Trigger parameter update"""
        rtio_output(self.trigger_channel, 0, 1)
        delay(1*us)

    @portable
    def frequency_to_ftw(self, freq: float) -> TInt32:
        ftw = int(freq * self.ftw_per_hz)
        return ftw

    @kernel
    def configure(self, frequency, amplitude, phase):
        """Configure DDS parameters"""
        ftw = self.frequency_to_ftw(frequency)
        amp = round(amplitude * 0x3FFF)
        ptw = round((phase % 360) / 360 * 0xFFFF)

        # Write all parameters
        # Amplitude components
        self.write_param_chunk(0, amp)
        self.write_param_chunk(1, 0)  # damp
        self.write_param_chunk(2, 0)
        self.write_param_chunk(3, 0)  # ddamp
        self.write_param_chunk(4, 0)
        self.write_param_chunk(5, 0)
        self.write_param_chunk(6, 0)  # dddamp
        self.write_param_chunk(7, 0)
        self.write_param_chunk(8, 0)

        # Phase/frequency components
        self.write_param_chunk(9, ptw)
        self.write_param_chunk(10, ftw & 0xFFFF)
        self.write_param_chunk(11, (ftw >> 16) & 0xFFFF)
        self.write_param_chunk(12, 0)  # chirp
        self.write_param_chunk(13, 0)

        self.trigger()

    # Individual parameter setters
    @kernel
    def set_frequency(self, freq):
        ftw = self.frequency_to_ftw(freq)
        self.write_param_chunk(10, (ftw >> 0) & 0xFFFF)
        self.write_param_chunk(11, (ftw >> 16) & 0xFFFF)
        self.trigger()

    @kernel
    def set_amplitude(self, amplitude):
        amp = round(amplitude * 0x3FFF)
        self.write_param_chunk(0, amp & 0xFFFF)
        self.trigger()

    @kernel
    def set_phase(self, phase):
        ptw = round((phase % 360) / 360 * 0xFFFF)
        self.write_param_chunk(9, ptw & 0xFFFF)
        self.trigger()