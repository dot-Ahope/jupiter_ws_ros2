#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import threading
import socket
import base64
import time
import sys

class ZedF9PRtkNode(Node):
    def __init__(self):
        super().__init__('zed_f9p_rtk_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('frame_id', 'gps_link')

        self.declare_parameter('ntrip_server', 'www.gnssdata.or.kr')
        self.declare_parameter('ntrip_port', 2101)
        self.declare_parameter('ntrip_user', 'geektrck@gmail.com')
        self.declare_parameter('ntrip_pass', 'gnss')
        self.declare_parameter('ntrip_mountpoint', 'SUWN-RTCM32')

        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value

        self.ntrip_server = self.get_parameter('ntrip_server').value
        self.ntrip_port = self.get_parameter('ntrip_port').value
        self.ntrip_user = self.get_parameter('ntrip_user').value
        self.ntrip_pass = self.get_parameter('ntrip_pass').value
        self.ntrip_mountpoint = self.get_parameter('ntrip_mountpoint').value

        # Publisher for GPS Fix
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)

        # Serial port with retry
        self.serial = None
        self._serial_lock = threading.Lock()
        self._serial_ok = False
        self._connect_serial()

        # Latest NMEA string to send back to caster
        self.last_gga = ""

        self.running = True

        # Start NMEA reading thread
        self.nmea_thread = threading.Thread(target=self.read_nmea)
        self.nmea_thread.daemon = True
        self.nmea_thread.start()

        # Start NTRIP client thread
        self.ntrip_thread = threading.Thread(target=self.ntrip_client)
        self.ntrip_thread.daemon = True
        self.ntrip_thread.start()

    def _connect_serial(self):
        """시리얼 포트 연결 시도. 실패 시 백그라운드에서 재시도."""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        except Exception:
            pass
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self._serial_ok = True
            self.get_logger().info(f"Opened serial port {self.port} at {self.baudrate} bps.")
        except Exception as e:
            self._serial_ok = False
            self.serial = None
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")

    def _try_reconnect(self):
        """시리얼 포트 재연결 시도 (lock 내부에서 호출)."""
        self._serial_ok = False
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        except Exception:
            pass
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self._serial_ok = True
            self.get_logger().info(f"Serial reconnected: {self.port}")
            return True
        except Exception as e:
            self.serial = None
            self.get_logger().warn(f"Serial reconnect failed: {e}")
            return False

    def ntrip_client(self):
        while self.running:
            try:
                self.get_logger().info(f"Connecting to NTRIP caster {self.ntrip_server}:{self.ntrip_port}...")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2.0) # Reduce timeout to allow periodic GGA sending
                s.connect((self.ntrip_server, self.ntrip_port))

                # Create HTTP Request
                auth_str = f"{self.ntrip_user}:{self.ntrip_pass}"
                b64_auth = base64.b64encode(auth_str.encode('ascii')).decode('ascii')
                
                request = f"GET /{self.ntrip_mountpoint} HTTP/1.1\r\nAuthorization: Basic {b64_auth}\r\n\r\n"
                
                s.sendall(request.encode('ascii'))
                
                # Wait for 200 OK
                response = s.recv(1024)
                if b"200 OK" not in response:
                    self.get_logger().error(f"NTRIP connection failed, response: {response}")
                    s.close()
                    time.sleep(5)
                    continue
                
                self.get_logger().info("Connected to NTRIP. Injecting RTCM to ZED-F9P...")

                last_gga_sent_time = 0
                last_report_time = time.time()
                bytes_received = 0

                while self.running:
                    try:
                        data = s.recv(4096)
                        if data:
                            if self._serial_ok and self.serial and self.serial.is_open:
                                try:
                                    self.serial.write(data)
                                except (serial.SerialException, OSError):
                                    self.get_logger().warn("Serial write failed during RTCM injection")
                                    with self._serial_lock:
                                        self._try_reconnect()
                            bytes_received += len(data)
                        else:
                            # Server closed connection cleanly
                            self.get_logger().warn("NTRIP Server closed the connection. Reconnecting...")
                            break
                    except socket.timeout:
                        pass # Normal timeout to allow periodic tasks

                    current_time = time.time()

                    # 1. Send GGA back to caster every 10 seconds to keep VRS alive
                    if current_time - last_gga_sent_time > 10.0 and self.last_gga:
                        try:
                            gga_packet = self.last_gga + "\r\n"
                            s.sendall(gga_packet.encode('ascii'))
                            last_gga_sent_time = current_time
                        except Exception:
                            pass # Socket write error, next recv will catch it

                    # 2. Report stats every 5 seconds
                    if current_time - last_report_time > 5.0:
                        if bytes_received > 0:
                            self.get_logger().info(f"Recv: {bytes_received} bytes of RTCM data in last 5s")
                            bytes_received = 0
                        last_report_time = current_time

            except ConnectionRefusedError:
                self.get_logger().warn("NTRIP connection refused. Reconnecting in 5s...")
                time.sleep(5)
            except Exception as e:
                self.get_logger().warn(f"NTRIP connection lost or error: {e}. Reconnecting in 5s...")
                time.sleep(5)

    def read_nmea(self):
        while self.running:
            if not self._serial_ok or self.serial is None:
                time.sleep(2)
                with self._serial_lock:
                    self._try_reconnect()
                continue
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                        self.last_gga = line
                        self.process_gga(line)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"Serial read error: {e}. Attempting reconnect...")
                with self._serial_lock:
                    self._try_reconnect()
                time.sleep(2)
            except Exception as e:
                # Ignore random parse errors
                pass

    def parse_latlon(self, dm, is_south_or_west):
        if not dm: return 0.0
        # Format is ddmm.mmmm or dddmm.mmmm
        dot_idx = dm.find('.')
        if dot_idx < 2: return 0.0
        deg_str = dm[:dot_idx-2]
        min_str = dm[dot_idx-2:]
        try:
            deg = float(deg_str)
            mins = float(min_str)
            val = deg + mins / 60.0
            if is_south_or_west in ('S', 'W'):
                val = -val
            return val
        except ValueError:
            return 0.0

    def process_gga(self, line):
        # Example format: $GNGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
        parts = line.split(',')
        if len(parts) < 10: return
        
        try:
            lat = self.parse_latlon(parts[2], parts[3])
            lon = self.parse_latlon(parts[4], parts[5])
            qual = int(parts[6]) if parts[6] else 0
            alt = float(parts[9]) if parts[9] else 0.0

            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = self.frame_id

            # GGA quality → NavSatStatus 매핑 (ublox ZED-F9P 기준)
            # NavSatStatus에는 -1, 0, 1, 2 네 값만 존재 (3은 없음)
            #   qual=4 → RTK Fixed  → STATUS_GBAS_FIX (2)  σ²=0.0004 (2cm급)
            #   qual=5 → RTK Float  → STATUS_SBAS_FIX (1)  σ²=0.25   (50cm급)
            #   qual=1,2 → 3D Fix   → STATUS_FIX (0)       σ²=6.25   (2.5m급)
            #   qual=0 → No Fix     → STATUS_NO_FIX (-1)
            # 참고: GBAS/SBAS는 원래 DGPS/WAAS를 의미하나, ROS 메시지에
            #       RTK 전용 상수가 없어 정밀도 순서에 맞춰 재활용함
            if qual == 4:
                fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            elif qual == 5:
                fix.status.status = NavSatStatus.STATUS_SBAS_FIX
            elif qual > 0:
                fix.status.status = NavSatStatus.STATUS_FIX
            else:
                fix.status.status = NavSatStatus.STATUS_NO_FIX

            fix.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_GALILEO

            fix.latitude = lat
            fix.longitude = lon
            fix.altitude = alt

            # RTK 품질별 공분산 설정
            if qual == 4:
                # RTK Fixed: 1~3cm 정밀도 → σ²=0.02²=0.0004
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                cov = 0.0004
                fix.position_covariance = [cov, 0.0, 0.0, 0.0, cov, 0.0, 0.0, 0.0, cov * 4.0]
            elif qual == 5:
                # RTK Float: 20cm~1m 정밀도 → σ²=0.5²=0.25
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                cov = 0.25
                fix.position_covariance = [cov, 0.0, 0.0, 0.0, cov, 0.0, 0.0, 0.0, cov * 4.0]
            elif qual > 0:
                # 3D Fix: ~2.5m 정밀도 → σ²=2.5²=6.25
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                cov = 6.25
                fix.position_covariance = [cov, 0.0, 0.0, 0.0, cov, 0.0, 0.0, 0.0, cov * 2.0]
            else:
                # No Fix: 매우 큰 공분산 (사실상 사용 불가)
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                cov = 9999.0
                fix.position_covariance = [cov, 0.0, 0.0, 0.0, cov, 0.0, 0.0, 0.0, cov]

            self.fix_pub.publish(fix)

        except Exception as e:
            pass

    def destroy_node(self):
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZedF9PRtkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
