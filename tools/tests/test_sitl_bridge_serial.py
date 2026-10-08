import unittest
from unittest.mock import patch

from tools.sitl_bridge import open_serial_port


class FakeSerial:
    def __init__(self):
        self.events = []
        self.port = None
        self.baudrate = None
        self.timeout = None
        self.dtr = None
        self.rts = None

    def __setattr__(self, name, value):
        if name in {"port", "baudrate", "timeout", "dtr", "rts"} and hasattr(self, "events"):
            self.events.append((name, value))
        object.__setattr__(self, name, value)

    def open(self):
        self.events.append(("open", None))


class SitlBridgeSerialTests(unittest.TestCase):
    def test_dtr_and_rts_are_disabled_before_open(self):
        fake = FakeSerial()
        with patch("tools.sitl_bridge.serial.Serial", return_value=fake):
            result = open_serial_port("COMx", 57600)

        self.assertIs(result, fake)
        self.assertLess(fake.events.index(("dtr", False)), fake.events.index(("open", None)))
        self.assertLess(fake.events.index(("rts", False)), fake.events.index(("open", None)))


if __name__ == "__main__":
    unittest.main()
