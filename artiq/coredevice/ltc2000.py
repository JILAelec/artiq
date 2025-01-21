from artiq.coredevice.rtio import rtio_output
from artiq.experiment import *
from artiq.coredevice import spi2
from artiq.language.core import kernel, delay
from artiq.language.units import us

class DDS:
    """Shuttler Core DDS spline.

    A Shuttler channel can generate a waveform `w(t)` that is the sum of a
    cubic spline `a(t)` and a sinusoid modulated in amplitude by a cubic
    spline `b(t)` and in phase/frequency by a quadratic spline `c(t)`, where

    .. math::
        w(t) = a(t) + b(t) * cos(c(t))

    and `t` corresponds to time in seconds.
    This class controls the cubic spline `b(t)` and quadratic spline `c(t)`,
    in which

    .. math::
        b(t) &= g * (q_0 + q_1t + \\frac{q_2t^2}{2} + \\frac{q_3t^3}{6})

        c(t) &= r_0 + r_1t + \\frac{r_2t^2}{2}

    `b(t)` is in volts, `c(t)` is in number of turns. Note that `b(t)`
    contributes to a constant gain of :math:`g=1.64676`.

    :param channel: RTIO channel number of this DC-bias spline interface.
    :param core_device: Core device name.
    """
    kernel_invariants = {"core", "channel", "target_o"}

    def __init__(self, dmgr, channel, core_device="core"):
        self.core = dmgr.get(core_device)
        self.channel = channel
        self.target_o = channel << 8

    @kernel
    def set_waveform(self, b0: TInt32, b1: TInt32, b2: TInt64, b3: TInt64,
            c0: TInt32, c1: TInt32, c2: TInt32):
        """Set the DDS spline waveform.

        Given `b(t)` and `c(t)` as defined in :class:`DDS`, the coefficients
        should be configured by the following formulae.

        .. math::
            T &= 8*10^{-9}

            b_0 &= q_0

            b_1 &= q_1T + \\frac{q_2T^2}{2} + \\frac{q_3T^3}{6}

            b_2 &= q_2T^2 + q_3T^3

            b_3 &= q_3T^3

            c_0 &= r_0

            c_1 &= r_1T + \\frac{r_2T^2}{2}

            c_2 &= r_2T^2

        :math:`b_0`, :math:`b_1`, :math:`b_2` and :math:`b_3` are 16, 32, 48
        and 48 bits in width respectively. See :meth:`shuttler_volt_to_mu` for
        machine unit conversion. :math:`c_0`, :math:`c_1` and :math:`c_2` are
        16, 32 and 32 bits in width respectively.

        Note: The waveform is not updated to the Shuttler Core until
        triggered. See :class:`Trigger` for the update triggering mechanism.

        :param b0: The :math:`b_0` coefficient in machine units.
        :param b1: The :math:`b_1` coefficient in machine units.
        :param b2: The :math:`b_2` coefficient in machine units.
        :param b3: The :math:`b_3` coefficient in machine units.
        :param c0: The :math:`c_0` coefficient in machine units.
        :param c1: The :math:`c_1` coefficient in machine units.
        :param c2: The :math:`c_2` coefficient in machine units.
        """
        coef_words = [
            b0,
            b1,
            b1 >> 16,
            b2 & 0xFFFF,
            (b2 >> 16) & 0xFFFF,
            (b2 >> 32) & 0xFFFF,
            b3 & 0xFFFF,
            (b3 >> 16) & 0xFFFF,
            (b3 >> 32) & 0xFFFF,
            c0,
            c1,
            c1 >> 16,
            c2,
            c2 >> 16,
        ]

        for i in range(len(coef_words)):
            rtio_output(self.target_o | i, coef_words[i])
            delay_mu(int64(self.core.ref_multiplier))


class Trigger:
    """Shuttler Core spline coefficients update trigger.

    :param channel: RTIO channel number of the trigger interface.
    :param core_device: Core device name.
    """
    kernel_invariants = {"core", "channel", "target_o"}

    def __init__(self, dmgr, channel, core_device="core"):
        self.core = dmgr.get(core_device)
        self.channel = channel
        self.target_o = channel << 8

    @kernel
    def trigger(self, trig_out):
        """Triggers coefficient update of (a) Shuttler Core channel(s).

        Each bit corresponds to a Shuttler waveform generator core. Setting
        ``trig_out`` bits commits the pending coefficient update (from
        ``set_waveform`` in :class:`DCBias` and :class:`DDS`) to the Shuttler Core
        synchronously.

        :param trig_out: Coefficient update trigger bits. The MSB corresponds
            to Channel 15, LSB corresponds to Channel 0.
        """
        rtio_output(self.target_o, trig_out)

class Clear:
    """Shuttler Core clear signal.

    :param channel: RTIO channel number of the clear interface.
    :param core_device: Core device name.
    """
    kernel_invariants = {"core", "channel", "target_o"}

    def __init__(self, dmgr, channel, core_device="core"):
        self.core = dmgr.get(core_device)
        self.channel = channel
        self.target_o = channel << 8

    @kernel
    def clear(self, clear_out):
        """Clears the Shuttler Core channel(s).

        Each bit corresponds to a Shuttler waveform generator core. Setting
        ``clear_out`` bits clears the corresponding channels in the Shuttler Core
        synchronously.

        :param clear_out: Clear signal bits. The MSB corresponds
            to Channel 15, LSB corresponds to Channel 0.
        """
        rtio_output(self.target_o, clear_out)

class Reset:
    """Shuttler Core reset signal.

    :param channel: RTIO channel number of the clear interface.
    :param core_device: Core device name.
    """
    kernel_invariants = {"core", "channel", "target_o"}

    def __init__(self, dmgr, channel, core_device="core"):
        self.core = dmgr.get(core_device)
        self.channel = channel
        self.target_o = channel << 8

    @kernel
    def reset(self, reset):
        """Resets the LTC2000 DAC.

        :param reset: Reset signal.
        """
        rtio_output(self.target_o, reset)

class Gain:
    """LTC2000 sub DDS gain control.

    Not yet fully implemented.
    """
    kernel_invariants = {"core", "channel", "target_o"}

    def __init__(self, dmgr, channel, core_device="core"):
        self.core = dmgr.get(core_device)
        self.channel = channel
        self.target_o = channel << 8

# class LTC2000:
#     def __init__(self, dmgr, channel, spi_device):
#         self.spi = dmgr.get(spi_device)
#         self.bus_channel = channel
#         self.ftw_per_hz = (2**32) / 2400e6
#         # channels for rtlink interface
#         self.data_channel = channel      # Parameter data writes
#         self.trigger_channel = channel+1 # Parameter trigger

#     @kernel
#     def init(self):
#         """Initialize SPI interface"""
#         config = (0 * spi2.SPI_OFFLINE |
#                  1 * spi2.SPI_END |
#                  0 * spi2.SPI_INPUT |
#                  0 * spi2.SPI_CS_POLARITY |
#                  0 * spi2.SPI_CLK_POLARITY |
#                  0 * spi2.SPI_CLK_PHASE |
#                  0 * spi2.SPI_LSB_FIRST |
#                  0 * spi2.SPI_HALF_DUPLEX)
#         self.spi.set_config_mu(config, 16, 32, 1)

#     @kernel
#     def write(self, addr, data):
#         """Write to LTC2000 registers via SPI"""
#         self.spi.write(((addr & 0x7F) << 24) | ((data & 0xFF) << 16))

#     @kernel
#     def read(self, addr):
#         """Read LTC2000 registers via SPI"""
#         return self.spi.write((1 << 31) | ((addr & 0x7F) << 24)) & 0xFF000000

#     @kernel
#     def initialize(self):
#         """Full initialization sequence for LTC2000"""
#         self.init()
#         # LTC2000 register configuration
#         self.write(0x01, 0x00)  # Reset, power down controls
#         self.write(0x02, 0x00)  # Clock and DCKO controls
#         self.write(0x03, 0x01)  # DCKI controls
#         self.write(0x04, 0x0B)  # Data input controls
#         self.write(0x05, 0x00)  # Synchronizer controls
#         self.write(0x07, 0x00)  # Linearization controls
#         self.write(0x08, 0x08)  # Linearization voltage controls
#         self.write(0x18, 0x00)  # LVDS test MUX controls
#         self.write(0x19, 0x00)  # Temperature measurement controls
#         self.write(0x1E, 0x00)  # Pattern generator enable
#         self.write(0x1F, 0x00)  # Pattern generator data

#     # Parameter interface methods
#     @kernel
#     def write_param_chunk(self, addr, data):
#         """Write 16-bit chunk of parameter data with 4-bit address"""
#         # Encode address and data together
#         word = ((addr & 0xF) << 16) | (data & 0xFFFF)
#         rtio_output(self.data_channel, word)
#         delay(1*us)

#     @kernel
#     def trigger(self):
#         """Trigger parameter update"""
#         rtio_output(self.trigger_channel, 1)
#         delay(1*us)
#         rtio_output(self.trigger_channel, 0)

#     @portable
#     def frequency_to_ftw(self, freq: float) -> TInt32:
#         ftw = int(freq * self.ftw_per_hz)
#         return ftw

#     @kernel
#     def configure(self, frequency, amplitude, phase):
#         """Configure DDS parameters"""
#         ftw = self.frequency_to_ftw(frequency)
#         amp = round(amplitude * 0x3FFF)
#         ptw = round((phase % 360) / 360 * 0xFFFF)

#         # Write all parameters
#         # Amplitude components
#         self.write_param_chunk(0, amp)
#         self.write_param_chunk(1, 0)  # damp
#         self.write_param_chunk(2, 0)
#         self.write_param_chunk(3, 0)  # ddamp
#         self.write_param_chunk(4, 0)
#         self.write_param_chunk(5, 0)
#         self.write_param_chunk(6, 0)  # dddamp
#         self.write_param_chunk(7, 0)
#         self.write_param_chunk(8, 0)

#         # Phase/frequency components
#         self.write_param_chunk(9, ptw)
#         self.write_param_chunk(10, ftw & 0xFFFF)
#         self.write_param_chunk(11, (ftw >> 16) & 0xFFFF)
#         self.write_param_chunk(12, 0)  # chirp
#         self.write_param_chunk(13, 0)

#         self.trigger()

#     # Individual parameter setters
#     @kernel
#     def set_frequency(self, freq):
#         ftw = self.frequency_to_ftw(freq)
#         self.write_param_chunk(10, (ftw >> 0) & 0xFFFF)
#         self.write_param_chunk(11, (ftw >> 16) & 0xFFFF)
#         self.trigger()

#     @kernel
#     def set_amplitude(self, amplitude):
#         amp = round(amplitude * 0x3FFF)
#         self.write_param_chunk(0, amp & 0xFFFF)
#         self.trigger()

#     @kernel
#     def set_phase(self, phase):
#         ptw = round((phase % 360) / 360 * 0xFFFF)
#         self.write_param_chunk(9, ptw & 0xFFFF)
#         self.trigger()