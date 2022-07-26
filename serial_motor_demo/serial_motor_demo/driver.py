import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import EncoderVals
import time
import math
import serial
from threading import Lock


class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')

        # Setup parameters

        self.declare_parameter('serial_port', value="/dev/ttyUSB0")
        self.serial_port = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=115200)
        self.baud_rate = self.get_parameter('baud_rate').value

        self.declare_parameter('serial_debug', value=False)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if (self.debug_serial_cmds):
            print("Serial debug enabled")

        # Setup topics & services

        self.subscription = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motor_command_callback,
            10)

        self.speed_pub = self.create_publisher(MotorVels, 'motor_vels', 10)

        # Member Variables

        self.last_enc_read_time = time.time()
        self.last_m1_enc = 0
        self.last_m2_enc = 0
        self.m1_spd = 0.0
        self.m2_spd = 0.0

        self.mutex = Lock()

        # Open serial comms

        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(
            self.serial_port, self.baud_rate, timeout=1.0)
        print(f"Connected to {self.conn}")

    # Raw serial commands

    def send_pwm_motor_command(self, mot_1_pwm, mot_2_pwm):
        self.send_command(f"o {int(mot_1_pwm)} {int(mot_2_pwm)}")

    def send_feedback_motor_command(self, mot_1_req_rad_sec, mot_2_req_rad_sec):
        self.send_command(f"m {mot_1_req_rad_sec} {mot_2_req_rad_sec}")

    def send_encoder_read_command(self):
        resp = self.send_command(f"e")
        if resp:
            return [float(raw_enc) for raw_enc in resp.split()]
        return []

    # More user-friendly functions

    def motor_command_callback(self, motor_command):
        if (motor_command.is_pwm):
            self.send_pwm_motor_command(
                motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec)
        else:
            self.send_feedback_motor_command(
                motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec)

    def check_encoders(self):
        resp = self.send_encoder_read_command()
        if (resp):
            spd_msg = MotorVels()
            spd_msg.mot_1_rad_sec = self.resp[0]
            spd_msg.mot_2_rad_sec = self.resp[1]
            self.speed_pub.publish(spd_msg)

    # Utility functions

    def send_command(self, cmd_string):

        self.mutex.acquire()
        try:
            cmd_string += "\n"
            self.conn.write(cmd_string.encode("utf-8"))
            if (self.debug_serial_cmds):
                print("Sent: " + cmd_string)

            # Adapted from original
            c = ''
            value = ''
            while c != '\n':
                c = self.conn.read(1).decode("utf-8")
                if (c == ''):
                    print("Error: Serial timeout on command: " + cmd_string)
                    return ''
                value += c

            value = value.strip('\n')

            if (self.debug_serial_cmds):
                print("Received: " + value)
            return value
        finally:
            self.mutex.release()

    def close_conn(self):
        self.conn.close()


def main(args=None):

    rclpy.init(args=args)

    motor_driver = MotorDriver()

    while rclpy.ok():
        rclpy.spin_once(motor_driver)
        # motor_driver.check_encoders()

    motor_driver.close_conn()
    motor_driver.destroy_node()
    rclpy.shutdown()
