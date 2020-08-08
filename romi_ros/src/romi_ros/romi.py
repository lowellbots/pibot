# Copyright Pololu Corporation.  For more information, see https://www.pololu.com/
# Merged with LSM6 by Dan Dugmore

import smbus
import struct
import time
import collections

class Regs(object):
  CTRL1_XL    = 0x10
  CTRL2_G     = 0x11
  CTRL3_C     = 0x12
  OUTX_L_G    = 0x22
  OUTX_L_XL   = 0x28

Vector = collections.namedtuple('Vector', 'x y z')

class LSM6(object):

  def __init__(self, slave_addr = 0b1101011):
    self.bus = smbus.SMBus(1)
    self.sa = slave_addr
    self.g = Vector(0, 0, 0)
    self.a = Vector(0, 0, 0)
      
  def enable(self):
    self.bus.write_byte_data(self.sa, Regs.CTRL1_XL, 0x50) # 208 Hz ODR, 2 g FS
    self.bus.write_byte_data(self.sa, Regs.CTRL2_G, 0x58) # 208 Hz ODR, 1000 dps FS
    self.bus.write_byte_data(self.sa, Regs.CTRL3_C, 0x04) # IF_INC = 1 (automatically increment register address)

  def read_gyro(self):
    byte_list = self.bus.read_i2c_block_data(self.sa, Regs.OUTX_L_G, 6)
    self.g = Vector(*struct.unpack('hhh', bytes(byte_list)))

  def read_accel(self):
    byte_list = self.bus.read_i2c_block_data(self.sa, Regs.OUTX_L_XL, 6)
    self.a = Vector(*struct.unpack('hhh', bytes(byte_list)))

  def read(self):
    self.read_gyro()
    self.read_accel()

class Romi:
  def __init__(self):
    self.bus = smbus.SMBus(1)
    self.imu = LSM6()

  def read_unpack(self, address, size, format):
    # Ideally we could do this:
    #    byte_list = self.bus.read_i2c_block_data(20, address, size)
    # But the AVR's TWI module can't handle a quick write->read transition,
    # since the STOP interrupt will occasionally happen after the START
    # condition, and the TWI module is disabled until the interrupt can
    # be processed.
    #
    # A delay of 0.0001 (100 us) after each write is enough to account
    # for the worst-case situation in our example code.

    self.bus.write_byte(20, address)
    time.sleep(0.0001)
    byte_list = [self.bus.read_byte(20) for _ in range(size)]
    return struct.unpack(format, bytes(byte_list))

  def write_pack(self, address, format, *data):
    data_array = list(struct.pack(format, *data))
    self.bus.write_i2c_block_data(20, address, data_array)
    time.sleep(0.0001)

  def leds(self, red, yellow, green):
    self.write_pack(0, 'BBB', red, yellow, green)

  def play_notes(self, notes):
    self.write_pack(24, 'B14s', 1, notes.encode("ascii"))

  def motors(self, left, right):
    self.write_pack(6, 'hh', left, right)
    self.watchdog()

  def motor(self, index, speed):
    if index in [0, 1]:
      self.write_pack(6 + (index * 2), 'h', speed)
      self.watchdog()
    
  def watchdog(self):
    self.write_pack(43, 'B', 1)

  def reset_encoders(self):
    self.write_pack(44, 'B', 1)

  def read_buttons(self):
    return self.read_unpack(3, 3, "???")

  def read_battery_millivolts(self):
    return self.read_unpack(10, 2, "H")

  def read_analog(self):
    return self.read_unpack(12, 12, "HHHHHH")

  def read_encoders(self):
    return self.read_unpack(39, 4, 'hh')

  def test_read8(self):
    self.read_unpack(0, 8, 'cccccccc')

  def test_write8(self):
    self.bus.write_i2c_block_data(20, 0, [0,0,0,0,0,0,0,0])
    time.sleep(0.0001)

  def beepBoop(self):
    self.play_notes("o4l16ceg>c8")

