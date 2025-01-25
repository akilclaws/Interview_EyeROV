import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import socket
import threading

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.declare_parameter('interval', 1000)  # Interval in milliseconds
        self.interval = self.get_parameter('interval').value

        # ROS publishers for sensor data
        self.voltage_pub = self.create_publisher(Float32, 'supply_voltage', 10)
        self.temp_pub = self.create_publisher(Float32, 'environment_temperature', 10)
        self.yaw_pub = self.create_publisher(Float32, 'yaw', 10)
        self.pitch_pub = self.create_publisher(Float32, 'pitch', 10)
        self.roll_pub = self.create_publisher(Float32, 'roll', 10)

        # ROS services for start and stop commands
        self.callback_group = ReentrantCallbackGroup()
        self.start_service = self.create_service(
            Int32, 'start_sensor', self.start_sensor, callback_group=self.callback_group
        )
        self.stop_service = self.create_service(
            Int32, 'stop_sensor', self.stop_sensor, callback_group=self.callback_group
        )

        # TCP communication setup
        self.host = '127.0.0.1'  # Replace with the sensor's IP
        self.port = 2000
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.listen_to_sensor, daemon=True)

        try:
            self.socket.connect((self.host, self.port))
            self.get_logger().info('Connected to the sensor.')
        except ConnectionError as e:
            self.get_logger().error(f'Failed to connect to sensor: {e}')
            return

        # Automatically send start command
        self.send_start_command(self.interval)
        self.thread.start()

    def send_start_command(self, interval):
        interval_bytes = interval.to_bytes(2, 'little')
        message = f'#03{interval_bytes.hex().upper()}\r\n'
        self.socket.sendall(message.encode('ascii'))
        self.get_logger().info(f'Sent start command with interval: {interval}ms')

    def send_stop_command(self):
        message = f'#09\r\n'
        self.socket.sendall(message.encode('ascii'))
        self.get_logger().info('Sent stop command.')

    def start_sensor(self, request, response):
        interval = request.data
        self.send_start_command(interval)
        response.success = True
        return response

    def stop_sensor(self, request, response):
        self.send_stop_command()
        response.success = True
        return response

    def listen_to_sensor(self):
        while not self.stop_event.is_set():
            try:
                data = self.socket.recv(1024).decode('ascii')
                if data.startswith('$11'):
                    self.decode_status_message(data)
            except Exception as e:
                self.get_logger().error(f'Error receiving data: {e}')

    def decode_status_message(self, message):
        try:
            # Remove start ('$11') and end ('\r\n')
            payload = message[3:-2]
            supply_voltage = int(payload[0:4], 16) / 1000.0  # Convert to volts
            env_temp = int(payload[4:8], 16) / 10.0  # Convert to Celsius
            yaw = int(payload[8:12], 16) / 10.0  # Convert to degrees
            pitch = int(payload[12:16], 16) / 10.0
            roll = int(payload[16:20], 16) / 10.0

            # Publish the decoded values
            self.voltage_pub.publish(Float32(data=supply_voltage))
            self.temp_pub.publish(Float32(data=env_temp))
            self.yaw_pub.publish(Float32(data=yaw))
            self.pitch_pub.publish(Float32(data=pitch))
            self.roll_pub.publish(Float32(data=roll))

            self.get_logger().info(
                f'Received Status: Voltage={supply_voltage}V, Temp={env_temp}C, Yaw={yaw}, Pitch={pitch}, Roll={roll}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to decode status message: {e}')

    def destroy_node(self):
        self.stop_event.set()
        self.send_stop_command()
        self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

