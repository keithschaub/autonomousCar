from flask import Flask, render_template
from flask_socketio import SocketIO
import random
import time
import threading

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)


def random_data_emitter():
    angle = 30  # Start angle
    while True:
        time.sleep(0.1)  # Adjust the sleep time as needed
        distance = 50 + random.randint(-5, 5)  # Distance around 50 cm with slight randomness
        #print(f"Emitting angle: {angle}, distance: {distance}")
        socketio.emit('newdata', {'angle': angle, 'distance': distance})

        angle += 5  # Increment angle
        if angle > 150:
            angle = 30  # Reset angle after reaching 150 degrees


@app.route('/')
def index():
    return render_template('index.html')


if __name__ == '__main__':
    threading.Thread(target=random_data_emitter).start()
    socketio.run(app, debug=True, allow_unsafe_werkzeug=True)
