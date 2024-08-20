import serial
import threading

def read_from_port(ser):
    while True:
        reading = ser.readline().decode('utf-8').strip()
        # Print the received data
        print(f"Received: {reading}")

def main():
    # Open the actual serial port (replace with your port and baud rate)
    ser = serial.Serial('/dev/ttyUSB0', 9600)

    # Create a thread to handle reading from the serial port
    thread = threading.Thread(target=read_from_port, args=(ser,))
    thread.start()

    # You can also write to the serial port if needed
    while True:
        data_to_send = input("Enter data to send to the serial port: ")
        # Send the data to the Arduino
        ser.write(data_to_send.encode('utf-8') + b'\n')
        # Print the data that was sent
        print(f"Sent: {data_to_send}")

if __name__ == "__main__":
    main()
