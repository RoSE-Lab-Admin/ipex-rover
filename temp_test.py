import time
from smbus2 import SMBus

BUS = 1
AVIONICS_ADDR = 0x18
BATTERY_ADDR = 0x19
TEMP_REG = 0x05

bus = SMBus(BUS)

def read_temp_c(addr):
    data = bus.read_i2c_block_data(addr, TEMP_REG, 2)

    raw = (data[0] << 8) | data[1]

    temp_c = (raw & 0x0FFF) / 16.0

    if raw & 0x1000:
        temp_c -= 256.0

    return temp_c

while True:
    avionics_temp = read_temp_c(AVIONICS_ADDR)
    battery_temp = read_temp_c(BATTERY_ADDR)

    print(
        f"Avionics: {avionics_temp:.2f} C   "
        f"Battery: {battery_temp:.2f} C"
    )

    time.sleep(1)
