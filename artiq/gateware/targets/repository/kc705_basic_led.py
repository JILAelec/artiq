from artiq.experiment import *


class LED(EnvExperiment):
    def build(self):
        self.setattr_device("core")
        self.setattr_device("ttl0")
        self.setattr_device("ttl1")

    @kernel
    def run(self):
        self.core.reset()
        self.ttl0.on()
        self.ttl1.on()
