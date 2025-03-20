import machine
import time

class DS3231:
    def __init__(self, i2c):
        self.i2c = i2c
        self.addr = 0x68

    def _bcd_to_dec(self, bcd):
        return (bcd // 16 * 10) + (bcd % 16)

    def _dec_to_bcd(self, dec):
        return (dec // 10 * 16) + (dec % 10)

    def get_time(self):
        data = self.i2c.readfrom_mem(self.addr, 0x00, 7)
        return (
            self._bcd_to_dec(data[6] + 2000), # Year
            self._bcd_to_dec(data[5]),        # Month
            self._bcd_to_dec(data[4]),        # Day
            self._bcd_to_dec(data[2]),        # Hour
            self._bcd_to_dec(data[1]),        # Minute
            self._bcd_to_dec(data[0]),        # Second
        )

    def set_time(self, year, month, day, hour, minute, second):
        self.i2c.writeto_mem(self.addr, 0x00, bytearray([
            self._dec_to_bcd(second),
            self._dec_to_bcd(minute),
            self._dec_to_bcd(hour),
            0,
            self._dec_to_bcd(day),
            self._dec_to_bcd(month),
            self._dec_to_bcd(year - 2000)
        ]))