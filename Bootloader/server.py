import serial
import time
import os  

# Serial port configuration
SERIAL_PORT = "/dev/ttyACM0"  # Adjust to match your STM32 serial port
BAUD_RATE = 115200            # Baud rate should match the STM32 configuration
VERSION_NUMBER = "1.1"
UPDATE_AVAILABLE = "200"
NO_UPDATE_AVAILABLE = "404"
FILE_SIZE_REQ = "File Size"
CHUNK_SIZE = 508
ACKNOWLEDGEMENT = "201"
ERROR_CODE = "500"
ENCODER = "utf-8"
POLYNOMIAL = 0x04C11DB7

file_path = os.path.join(os.path.dirname(__file__), "src", "compile", "build", "final.bin")
# For test 1
# file_path = os.path.join(os.path.dirname(__file__), "Test", "final1.bin") 
# For test 2
# file_path = os.path.join(os.path.dirname(__file__), "Test", "final2.bin") 
# For test 3
# file_path = os.path.join(os.path.dirname(__file__), "Test", "final3.bin") 
# For test 4
# file_path = os.path.join(os.path.dirname(__file__), "Test", "final4.bin") 

# Open the serial port
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)


def crc32_manual(data) -> int:
    crc = 0xFFFFFFFF

    for word in data:
        crc = crc ^ (word)

        for _ in range (32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ POLYNOMIAL)
            else:
                crc = crc<<1
            crc &= 0xFFFFFFFF

    return crc 

def compare_versions(version_request: str, current_version: str) -> int:
    """
    Compares two version numbers.

    :param version_request: The version being requested (e.g., "1.10").
    :param current_version: The current version (e.g., "1.2").
    :return: -1 if version_request < current_version,
             0 if version_request == current_version,
             1 if version_request > current_version.
    """
    # Split the version strings into major and minor components
    request_parts = [int(part) for part in version_request.split('.')]
    current_parts = [int(part) for part in current_version.split('.')]

    
    # Compare major and minor parts
    for req, curr in zip(request_parts, current_parts):
        if req < curr:
            return -1
        elif req > curr:
            return 1
    
    # If one version has more parts (e.g., "1.2.0" vs "1.2"), consider remaining parts
    if len(request_parts) < len(current_parts):
        return -1
    elif len(request_parts) > len(current_parts):
        return 1

    return 0  # Versions are equal



def main():
    print("Serial server is running...")

    try:
        while True:
            # Read incoming data from STM32
            update_request = ser.readline().decode().strip()

            if update_request:
                comparison = compare_versions(update_request, VERSION_NUMBER)
                print(f"Received message from STM32: Bootloader Version- {update_request}")
                
                # Check if the message is an update request
                if comparison >= 0:
                    ser.write(NO_UPDATE_AVAILABLE.encode(ENCODER))
                    print(f"Sent response: {NO_UPDATE_AVAILABLE}")
                    break
                else:
                    ser.write(UPDATE_AVAILABLE.encode(ENCODER))
                    print(f"Sent response: {UPDATE_AVAILABLE}")

            file_size_request = ser.readline().decode().strip()

            if file_size_request == FILE_SIZE_REQ:
                print(f"Received message from STM32: {file_size_request}")
                try:
                    with open(file_path, "rb") as file:
                        file_size = os.path.getsize(file_path)
                        ser.write(file_size.to_bytes(4, byteorder="big"))
                        print(f"Sent file size: {file_size}")

                        # Wait for the start signal
                        start_req = ser.readline().decode().strip()
                        if start_req == "START":
                            print("Starting file transfer...")
                            file.seek(0)  # Ensure we start at the beginning of the file
                            while chunk := file.read(CHUNK_SIZE):
                                # Compute CRC for the chunk
                                computed_crc= crc32_manual(chunk)
                                # Convert CRC to bytes (4 bytes, big-endian)
                                crc_bytes = computed_crc.to_bytes(4, byteorder="big")
                                # Append CRC bytes to the chunk
                                packet = chunk + crc_bytes
                                # Send the packet
                                ser.write(packet)

                                print(f"Sent chunk of {len(chunk)} bytes")
                                print(f"Sent packet of {len(packet)} bytes")
                                print(f"Sent CRC {computed_crc}")

                                # Wait for acknowledgment or error
                                response = ser.readline().decode().strip()
                                if response == ACKNOWLEDGEMENT:
                                    print("Chunk acknowledged by STM32.")
                                elif response == ERROR_CODE:
                                    print("CRC error reported by STM32. Resending chunk...")
                                    ser.write(b"Error")
                                    debug=ser.readline().decode().strip()
                                    print(f"Computed CRC by STM32 {debug}")
                                    file.seek(file.tell() - len(chunk))  # Rewind the file pointer
                                    continue  # Resend the same chunk
                                else:
                                    print(f"Unexpected response: {response}")
                                    break
                            
                            ser.write(VERSION_NUMBER.encode(ENCODER))
                            print("File transfer finished!")
                            break
                except FileNotFoundError:
                    print("File not found, unable to proceed with update.")
                    continue

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Server stopped by Ctrl+C")
        ser.close()
        print("port closed")

    try:
        while True:
            update_request = ser.readline().decode().strip()
            
            if update_request:
                print(f"Received message from STM32: {update_request}")
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Server stopped by Ctrl+C")
        
    finally:
        # Close the serial port on exit
        ser.close()
        print("port closed")

if __name__ == '__main__':
    main()
