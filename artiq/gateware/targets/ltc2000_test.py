from migen import *
from migen.build.generic_platform import *
from migen.fhdl.verilog import convert

from ltc2000 import LTC2000

from efc import ltc2000_pads
from misoc.targets.efc import BaseSoC

class Top(Module):
    def __init__(self, platform):
        self.submodules.ltc2000 = LTC2000(platform, ltc2000_pads)

        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_rio = ClockDomain()
        self.clock_domains.cd_sys2x = ClockDomain()
        self.clock_domains.cd_sys6x = ClockDomain()

if __name__ == "__main__":
    soc = BaseSoC()
    platform = soc.platform

    platform.add_extension(ltc2000_pads)
    top = Top(platform)

    ltc = platform.request("ltc2000", 0)
    ios = set()

    # Add each signal from the LTC2000 interface to ios
    for name in dir(ltc):
        if not name.startswith('_'):
            signal = getattr(ltc, name)
            if isinstance(signal, Signal):
                ios.add(signal)

    convert(top, name="top", ios=ios).write("top.v")