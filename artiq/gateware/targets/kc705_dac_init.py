from artiq.experiment import *
from artiq.coredevice import spi2 as spi

LTC_SPI_CONFIG = (0*spi.SPI_OFFLINE | 1*spi.SPI_END |
                    0*spi.SPI_INPUT | 0*spi.SPI_CS_POLARITY |
                    0*spi.SPI_CLK_POLARITY | 0*spi.SPI_CLK_PHASE |
                    0*spi.SPI_LSB_FIRST | 0*spi.SPI_HALF_DUPLEX)

LTC_SPI_DIV = 32
LTC_CS = 1 << 0
#LTC_CS = 0

class LED(EnvExperiment):
    def build(self):
        self.setattr_device("core")
        self.setattr_device("spi_ltc")
        self.setattr_device("ttl1")

    @kernel
    def run(self):
        self.core.reset()
        self.ttl1.on()
        self.spi_ltc.set_config_mu(
            LTC_SPI_CONFIG, 16, LTC_SPI_DIV, LTC_CS)
        self.spi_ltc.write(0xAAAA0000)
        delay(0.5)
        self.ttl1.off()

