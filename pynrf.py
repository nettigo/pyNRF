#!/usr/bin/env python3

import time
import struct

import RPi.GPIO as gpio
import spidev

class NRFProtocol (object):

    def __init__(self, bus=0, device=0):
        self.spi = spidev.SpiDev()
        #self.spi.max_speed_hz = 10000000
        self.spi.open(bus, device)

    def command_write(self, command, values):
        if not isinstance(values, bytes):
            raise ValueError(bytes)

        data = struct.pack('B', command) + values
        return bytes(self.spi.xfer2(list(data)))

    def command_read(self, command, size):
        values = b'\0' * size
        return self.command_write(command, values)

    def set_register(self, register, value):
        command = 0x20 | register
        values = struct.pack('B', value)
        response = self.command_write(command, values)
        status = response[0]
        return status

    def get_register(self, register):
        command = register
        response = self.command_read(command, 1)
        return response

    def set_address(self, register, address):
        if not isinstance(address, bytes):
            raise ValueError(bytes)

        command = 0x20 | register
        response = self.command_write(command, address)
        status = response[0]
        return status

class NRF24L01P (object):

    def __init__(self, ce_pin=22):
        self.ce_pin = ce_pin
        gpio.setmode(gpio.BOARD)
        gpio.setup(ce_pin, gpio.OUT)
        gpio.output(ce_pin, gpio.LOW)

        self.device = NRFProtocol()

    def __del__(self):
        gpio.cleanup(self.ce_pin)

    def activate(self):
        gpio.output(self.ce_pin, gpio.HIGH)

    def deactivate(self):
        gpio.output(self.ce_pin, gpio.LOW)

    def pulse(self):
        gpio.output(self.ce_pin, gpio.HIGH)
        gpio.output(self.ce_pin, gpio.LOW)

    def set_tx_address(self, address):
        self.device.set_address(0x10, address)

    def set_rx_address(self, pipe, address):
        register = 0x0a + pipe
        self.device.set_address(register, address)

    def set_channel(self, channel):
        self.device.set_register(0x05, channel)

    def set_payload_size(self, payload):
        self.device.set_register(0x12, payload)

    def get_payload_size(self):
        response = self.device.command_read(0x60, 1)
        payload = response[1]
        return payload

    def get_tx_status(self):
        response = self.device.get_register(0x07)
        status = response[1]
        if status & 0b00010000 != 0:
            return 2

        if status & 0b00100000 != 0:
            return 1

        return 0

    def get_rx_status(self):
        response = self.device.get_register(0x17)
        status = response[0]
        fifo_status = response[1]
        if fifo_status & 0b00000001 == 0:
            return True

        if status & 0b01000000 != 0:
            return True

        return False

    def reset_rx_status(self):
        self.device.set_register(0x07, 0b01000000)

    def reset_tx_status(self):
        self.device.set_register(0x07, 0b00110000)

    def tx_mode(self):
        self.device.set_register(0x0, 0b00001010)

    def rx_mode(self):
        self.device.set_register(0x0, 0b00001011)

    def power_off(self):
        self.set_register(0x0, 0)

    def flush_tx_buffer(self):
        self.device.command_read(0b11100001, 1)

    def flush_rx_buffer(self):
        self.device.command_read(0b11100010, 1)

    def write(self, data):
        self.device.command_write(0b10100000, data)

    def read(self, size):
        response = self.device.command_read(0b01100001, size)
        data = response[1:]
        return data


class Radio (object):

    def __init__(self, address, channel=1):
        self.device = NRF24L01P()
        self.device.set_channel(channel)
        self.device.set_rx_address(1, address)
        self._is_rx_mode = False

    def write(self, address, data):
        self.device.deactivate()
        self.device.tx_mode()
        self._is_rx_mode = False
        self.device.flush_tx_buffer()
        self.device.set_tx_address(address)
        self.device.set_rx_address(0, address)
        data_size = len(data)
        self.device.set_payload_size(data_size)
        self.device.write(data)
        self.device.reset_tx_status()
        self.device.pulse()

    def _rx_mode(self):
        self.device.deactivate()
        self.device.rx_mode()
        self.device.flush_rx_buffer()
        self.device.reset_rx_status()
        self.device.activate()
        self._is_rx_mode = True

    def read(self):
        data_size = self.device.get_payload_size()
        data = self.device.read(data_size)
        self.device.reset_rx_status()
        return data

    def available(self):
        if not self._is_rx_mode:
            self._rx_mode()

        is_data_available_to_read = self.device.get_rx_status()
        return is_data_available_to_read

    def flush(self):
        is_data_delivered = self.device.get_tx_status()
        return is_data_delivered
