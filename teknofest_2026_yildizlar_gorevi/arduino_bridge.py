#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import time

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        port = self.get_parameter('serial_port').value
        self.ser = None
        try:
            self.ser = serial.Serial(
                port, 115200, timeout=0.01, write_timeout=0.5)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f'✅ Arduino Bağlandı: {port}')
        except Exception as e:
            self.get_logger().error(f'❌ Bağlantı Hatası: {e}')
            return
        self.create_subscription(
            Int32MultiArray, '/auv/motor_pwm',
            self.motor_cb, 10)
        self.get_logger().info('📡 Arduino bridge hazır')

    def motor_cb(self, msg):
        if len(msg.data) != 8:
            return
        if self.ser is None or not self.ser.is_open:
            return
        cmd = ','.join(map(str, msg.data)) + '\n'
        try:
            self.ser.write(cmd.encode())
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f'❌ Serial yazma hatası: {e}')

    def stop_motors(self):
        if self.ser is None:
            return
        try:
            stop = '1500,1500,1500,1500,1500,1500,1500,1500\n'
            for _ in range(10):
                self.ser.write(stop.encode())
                self.ser.flush()
                time.sleep(0.05)
            self.ser.close()
        except:
            pass
        self.get_logger().info('🛑 Motorlar durduruldu')

def main():
    # ⏳ Güç verildikten sonra (Sensor Node'dan 15 sn sonra) toplam 35 saniye bekleme modu
    print("⏳ Sensor Node'un oturması bekleniyor... Arduino Bridge 35 saniye sonra başlayacak...")
    time.sleep(0)

    rclpy.init()
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_motors()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
