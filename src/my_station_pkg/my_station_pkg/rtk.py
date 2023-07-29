import rclpy
from rclpy.node import Node
from mavsdk import System, rtk
from mavsdk.mission import (MissionItem, MissionPlan)
from more_interfaces.srv import Comms, Trigger
from more_interfaces.msg import Trials1
from decimal import Decimal
from mavsdk.offboard import Attitude, AttitudeRate
import threading
from rclpy.executors import MultiThreadedExecutor
import asyncio
import serial
import time


PREAMBLE_RTCM = 0xD3
PREAMBLE_UBX = 0xB5
PREAMBLE_NMEA = 0x24

class UBXParser(Node):
    def __init__(self):
        self.debug = False

    def read_packet(self, dev, preamble):
        ubx_sync_byte = dev.read(1)
        if len(ubx_sync_byte) == 0 or ord(ubx_sync_byte) != 0x62:
            return None
        ubx_class_id = dev.read(1)
        ubx_message_id = dev.read(1)
        length_lsb = dev.read(1)
        length_msb = dev.read(1)
        pkt_len = (ord(length_msb) << 8) + ord(length_lsb)
        pkt = dev.read(pkt_len)
        checksum = dev.read(2)

        full_data = preamble + ubx_sync_byte
        full_data += ubx_class_id + ubx_message_id
        full_data += length_lsb + length_msb
        full_data += pkt + checksum

        if self.debug:
            print("UBX class id {} and message id {}".format(
                ord(ubx_class_id),
                ord(ubx_message_id))
            )

        # The ubx message in full_data could be parsed with pyubx2.UBXReader
        return full_data
    
class RTCMParser(Node):
    def __init__(self):
        self.debug = True
        self.crc_init = 0x01864cfb
        self._init_crc_table()

    def _init_crc_table(self):
        self.crc_table = [0] * 256
        for i in range(256):
            self.crc_table[i] = i << 16
            for j in range(8):
                self.crc_table[i] <<= 1
                if self.crc_table[i] & 0x1000000:
                    self.crc_table[i] ^= self.crc_init

    def crc24(self, msg):
        crc = 0
        for byte in msg:
            crc = ((crc << 8) & 0xFFFFFF) ^ self.crc_table[(crc >> 16) ^ byte]
        return crc

    def read_packet(self, dev, preamble):
        length_msb = dev.read(1)
        length_lsb = dev.read(1)
        pkt_len = ((ord(length_msb) & 0x3) << 8) + ord(length_lsb)
        pkt = dev.read(pkt_len)
        checksum = dev.read(3)

        full_data = preamble + length_msb + length_lsb + pkt + checksum
        rtcm_id = (pkt[0] << 4) + ((pkt[1] & 0xF0) >> 4)
        parity = (checksum[0] << 16) + (checksum[1] << 8) + checksum[2]
        if parity == self.crc24(full_data[:-3]):
            if self.debug:
                print("Got RTCM message with ID {}".format(rtcm_id))
            return full_data
        else:
            if self.debug:
                print("RTCM parity check failed. Skip message")
            return None





class RTKNode(Node):
    def __init__(self):
        super().__init__('rtk_node')
        self.drone = System()
        self.ublox = serial.Serial("/dev/ttyACM0", 9600)
        self.rtcm_parser = RTCMParser()
        self.ubx_parser = UBXParser()
        
    async def connect_drone(self):
        await self.drone.connect(system_address ="serial:///dev/ttyUSB2:57600")
        print("connected")

    async def send_rtcm(self):
        self.get_logger().info("hhssh")
        while True:
            try:
                preamble = self.ublox.read(1)
                print('1')
                if len(preamble) == 0:
                    continue
                if ord(preamble) == PREAMBLE_RTCM:
                    print('2')
                    rtcm_correction_data = self.rtcm_parser.read_packet(self.ublox, preamble)
                    if rtcm_correction_data is None:
                        print('3')
                        continue
                    try:
                        success = self.drone.rtk.send_rtcm_data(
                            rtk.RtcmData(str(rtcm_correction_data)))
                    except Exception as e:
                        self.get_logger().error(f"Drone connection failed: {str(e)}")
                    else:
                        print('rtcm sent')
                elif ord(preamble) == PREAMBLE_UBX:
                    print('4')
                    ubx = self.ubx_parser.read_packet(self.ublox, preamble)
                    if ubx is None:
                        print('5')
                        continue
                elif ord(preamble) == PREAMBLE_NMEA:
                    pass
            except Exception as e:
                self.get_logger().info(f"Exception: {e}")

def run_asyncio_loop(node):
    asyncio.run(node.connect_drone())
    asyncio.run(node.send_rtcm())

def main(args=None):
    rclpy.init(args=args)
    node = RTKNode()
    executor = MultiThreadedExecutor()

    # Run asyncio tasks in one thread
    thread = threading.Thread(target=run_asyncio_loop, args=(node,))
    thread.start()

    # Spin ROS2 node in another thread
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        # Shutdown and join the thread on exception
        executor.shutdown()
        thread.join()

if __name__ == '__main__':
    main()
