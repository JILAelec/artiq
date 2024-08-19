from artiq.experiment import *

class LTC2000Experiment(EnvExperiment):
    def build(self):
        self.setattr_device("core")
        self.setattr_device("ttl1")
        self.setattr_device("ltc2000")

    @kernel
    def run(self):
        self.core.reset()
        self.ttl1.on()

        # Initialize and configure the LTC2000
        self.ltc2000.initialize()
        self.ltc2000.configure(frequency_mhz=200.0)  # Starting at 200 MHz

        # Example of changing frequency
        for i in range(10):
            frequency_mhz = 100.0 + i * 10  # 100MHz to 190MHz
            ftw = self.ltc2000.frequency_to_ftw(frequency_mhz)
            self.ltc2000.ftw.write(ftw)
            delay(50*ms)

        self.ttl1.off()