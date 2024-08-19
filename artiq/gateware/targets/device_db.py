# This is an example device database that needs to be adapted to your setup.
# The list of devices here is not exhaustive.

core_addr = "192.168.1.2"

device_db = {
    # Core device
    "core": {
        "type": "local",
        "module": "artiq.coredevice.core",
        "class": "Core",
        "arguments": {
            "host": core_addr,
            "ref_period": 1e-9,
            "analyzer_proxy": "core_analyzer"
        }
    },
    "core_log": {
        "type": "controller",
        "host": "::1",
        "port": 1068,
        "command": "aqctl_corelog -p {port} --bind {bind} " + core_addr
    },
    "core_moninj": {
        "type": "controller",
        "host": "::1",
        "port_proxy": 1383,
        "port": 1384,
        "command": "aqctl_moninj_proxy --port-proxy {port_proxy} --port-control {port} --bind {bind} " + core_addr
    },
    "core_analyzer": {
        "type": "controller",
        "host": "::1",
        "port_proxy": 1385,
        "port": 1386,
        "command": "aqctl_coreanalyzer_proxy --port-proxy {port_proxy} --port-control {port} --bind {bind} " + core_addr
    },
    "core_cache": {
        "type": "local",
        "module": "artiq.coredevice.cache",
        "class": "CoreCache"
    },
    "core_dma": {
        "type": "local",
        "module": "artiq.coredevice.dma",
        "class": "CoreDMA"
    },

    "i2c_switch": {
        "type": "local",
        "module": "artiq.coredevice.i2c",
        "class": "I2CSwitch"
    },

    # Generic TTL
    "ttl0": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLOut",
        "arguments": {"channel": 0},
        "comment": "This is a fairly long comment, shown as tooltip."
    },
    "ttl1": {
        "type": "local",
        "module": "artiq.coredevice.ttl",
        "class": "TTLOut",
        "arguments": {"channel": 1},
        "comment": "Hello World"
    },

    # SPI master for LTC2000
    "spi_ltc": {
        "type": "local",
        "module": "artiq.coredevice.spi2",
        "class": "SPIMaster",
        "arguments": {"channel": 2}
    },

    # LTC2000 DAC
    "ltc2000": {
        "type": "local",
        "module": "artiq.coredevice.ltc",
        "class": "LTC2000",
        "arguments": {
            "spi_device": "spi_ltc",
            "csr_base": 0xe0008800
        }
    },
}
