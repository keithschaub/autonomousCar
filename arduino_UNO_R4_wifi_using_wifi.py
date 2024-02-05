import socket
import keyboard

# Replace with the IP address of your Arduino
arduino_ip = "192.168.86.42"
arduino_port = 80

print("hello")

def on_key_event(e):
    if e.name == 'a':
        s.sendall('A'.encode('utf-8'))
        print("Sent 'A' to arduino")
    elif e.name == 'up':
        s.sendall('F'.encode('utf-8'))      # can use this to move car forward
        print("Sent UP ARROW to arduino");
    elif e.name == 'down':
        s.sendall('B'.encode('utf-8'))      # can use this to move car forward
        print("Sent DOWN ARROW to arduino");
    elif e.name == 'left':
        s.sendall('L'.encode('utf-8'))      # can use this to move car forward
        print("Sent LEFT ARROW to arduino");
    elif e.name == 'right':
        s.sendall('R'.encode('utf-8'))      # can use this to move car forward
        print("Sent RIGHT ARROW to arduino");



with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((arduino_ip, arduino_port))

    print('connection established')

    # Register the event listener
    keyboard.on_press(on_key_event)

    buffer = ""
    while True:
        data = s.recv(1024).decode('utf-8')
        #print(data)
        if not data:
            break
        buffer += data
        while '\n' in buffer:
            complete_message, buffer = buffer.split('\n', 1)
            if complete_message.strip():
                print(f"Received: {complete_message}")

    print("Connection closed")
    keyboard.unhook_all()
