from artiq.experiment import *
from artiq.coredevice.ltc2000 import LTC2000

class DAC_Init(EnvExperiment):
    def build(self):
        self.setattr_device("core")
        self.setattr_device("ttl0")
        self.setattr_device("spi_ltc")

    def prepare(self):
        self.ltc2000 = LTC2000(self, "spi_ltc")
        self.frequency = 200e6  # 200 MHz
        self.amplitude = 0.9  # 90% of full scale
        self.phase = 0  # Starting phase

    @kernel
    def run(self):
        self.core.reset()
        self.ttl0.output()

        # Initialize the LTC2000
        self.ltc2000.initialize()

        # Configure the LTC2000
        self.ltc2000.configure(self.frequency, self.amplitude, self.phase)
