from flask import Flask, render_template
from flask_socketio import SocketIO
import random
import time
import threading

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

def random_data_emitter():
    while True:
        time.sleep(1)  # Simulating data receiving interval
        angle = random.randint(0, 360)
        distance = random.randint(1, 100)
        socketio.emit('newdata', {'angle': angle, 'distance': distance})

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    threading.Thread(target=random_data_emitter).start()
    socketio.run(app, debug=True, allow_unsafe_werkzeug=True)
