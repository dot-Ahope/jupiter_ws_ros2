import serial
import time
import binascii

def read_serial(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Opened {port} at {baudrate}")
        
        cmd = [0xFF, 0xFC, 0x05, 0x01, 0x01, 0x00, 0x07]
        ser.write(bytearray(cmd))
        print("Sent auto report command")

        start_time = time.time()
        buffer = bytearray()
        while time.time() - start_time < 5:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                buffer.extend(data)
                
                # Process buffer
                while len(buffer) >= 4:
                    # Look for header FF FB
                    if buffer[0] == 0xFF and buffer[1] == 0xFB:
                        length = buffer[2]
                        if len(buffer) >= length + 2: # +2 for FF FB? No, length usually includes checksum or payload?
                            # In Rosmaster_Lib:
                            # ext_len = bytearray(self.ser.read())[0]
                            # ext_type = bytearray(self.ser.read())[0]
                            # check_sum = ext_len + ext_type
                            # data_len = ext_len - 2
                            # ...
                            # So packet structure: FF FB LEN TYPE DATA... CHECKSUM
                            # Total length from LEN byte = LEN + 2 (LEN byte + TYPE byte + DATA + CHECKSUM? No)
                            # Let's check code:
                            # ext_len read.
                            # ext_type read.
                            # data_len = ext_len - 2.
                            # read data_len bytes.
                            # read checksum (rx_check_num).
                            # So total bytes after FB = 1 (LEN) + 1 (TYPE) + (LEN-2) (DATA) + 1 (CHECKSUM)?
                            # Wait. 1 + 1 + LEN - 2 = LEN.
                            # So LEN includes TYPE and DATA?
                            # And then CHECKSUM is extra?
                            # "while len(ext_data) < data_len: ... rx_check_num = value"
                            # It seems checksum is the LAST byte of the data read loop?
                            # "if len(ext_data) == data_len: rx_check_num = value"
                            # This implies data_len INCLUDES checksum?
                            # Let's re-read carefully.
                            # data_len = ext_len - 2
                            # loop until len(ext_data) == data_len.
                            # inside loop: read value. append to ext_data.
                            # if len(ext_data) == data_len: rx_check_num = value.
                            # So the last byte read IS the checksum.
                            # So ext_data includes checksum at the end?
                            # No, "rx_check_num = value".
                            # And "check_sum = check_sum + value" is in else.
                            # So checksum is NOT included in sum.
                            # So total bytes to read after LEN byte = 1 (TYPE) + (LEN-2) (DATA+CS).
                            # So total packet size = 2 (FF FB) + 1 (LEN) + 1 (TYPE) + (LEN-2) = 2 + LEN.
                            
                            packet_len = 2 + length
                            if len(buffer) >= packet_len:
                                packet = buffer[:packet_len]
                                packet_type = packet[3]
                                packet_data = packet[4:-1]
                                print(f"Packet: Type={hex(packet_type)} Len={length} Data={binascii.hexlify(packet_data).decode('utf-8')}")
                                buffer = buffer[packet_len:]
                            else:
                                break # Wait for more data
                        else:
                            break # Wait for more data
                    else:
                        # Shift buffer
                        buffer.pop(0)
            time.sleep(0.01)
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

print("Testing /dev/myserial (ttyUSB1)...")
read_serial('/dev/myserial', 115200)

print("\nTesting /dev/ttyUSB0...")
read_serial('/dev/ttyUSB0', 115200)
