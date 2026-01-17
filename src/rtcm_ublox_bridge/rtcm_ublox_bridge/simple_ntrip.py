import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message
import socket
import base64
import time
import errno

class SimpleNtripClient(Node):
    def __init__(self):
        super().__init__('simple_ntrip_client')
        
        # Parameters
        self.declare_parameter('host', 'rts2.ngii.go.kr')
        self.declare_parameter('port', 2101)
        self.declare_parameter('mountpoint', 'VRS-RTCM31')
        self.declare_parameter('username', 'cjinwook94')
        self.declare_parameter('password', 'ngii')
        self.declare_parameter('rtcm_topic', '/rtcm')

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.mountpoint = self.get_parameter('mountpoint').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        topic_name = self.get_parameter('rtcm_topic').value

        self.publisher_ = self.create_publisher(Message, topic_name, 10)
        
        self.get_logger().info(f'Starting NTRIP Client: {self.host}:{self.port}/{self.mountpoint}')
        
        self.connect_and_stream()

    def connect_and_stream(self):
        while rclpy.ok():
            try:
                self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.s.settimeout(5)
                self.s.connect((self.host, self.port))
                
                # NTRIP Request
                auth_str = f"{self.username}:{self.password}"
                auth_b64 = base64.b64encode(auth_str.encode()).decode()
                
                headers = [
                    f"GET /{self.mountpoint} HTTP/1.0",
                    "User-Agent: NTRIP Python Client",
                    f"Authorization: Basic {auth_b64}",
                    "Accept: */*",
                    "Connection: close",
                    "", ""
                ]
                self.s.sendall("\r\n".join(headers).encode())
                
                # Read Header
                response = b""
                header_done = False
                while not header_done:
                    chunk = self.s.recv(1)
                    if not chunk: break
                    response += chunk
                    if b"\r\n\r\n" in response:
                        header_done = True
                
                if b"200 OK" not in response:
                    self.get_logger().error(f"NTRIP Auth Failed: {response.decode(errors='ignore')}")
                    self.s.close()
                    time.sleep(5)
                    continue

                self.get_logger().info("NTRIP Connected! Streaming...")
                
                # Send GGA periodically (Fake Seoul position if needed)
                # Some servers require this to start sending VRS data
                nmea_gga = "$GPGGA,000001,3733.99,N,12658.68,E,1,08,1.0,100.0,M,0.0,M,,*79\r\n"
                self.s.sendall(nmea_gga.encode())
                last_gga_time = time.time()
                
                while rclpy.ok():
                    # Check for GGA update
                    if time.time() - last_gga_time > 10:
                         try:
                             self.s.sendall(nmea_gga.encode())
                             last_gga_time = time.time()
                         except:
                             break

                    try:
                        data = self.s.recv(1024)
                        if not data: break
                        
                        # Publish ROS Message
                        msg = Message()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.message = list(data) # Convert bytes to list of ints
                        self.publisher_.publish(msg)
                        
                    except socket.timeout:
                        continue
                    except socket.error as e:
                        if e.errno == errno.EWOULDBLOCK:
                            continue
                        raise
                        
            except Exception as e:
                self.get_logger().warn(f"Connection lost/failed: {e}. Retrying in 5s...")
                time.sleep(5)
            finally:
                if hasattr(self, 's'): self.s.close()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNtripClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
