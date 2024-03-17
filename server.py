# NOTE: this doesnt account for anything specific to the bot (i.e. delays, random times, optimization needed)
# See both this console and the webs
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

app = Flask(__name__)
# unsafe key lol
app.config['SECRET_KEY'] = 'xxx'
socketio = SocketIO(app)


# realistically the data here is gonna be sent from the bot.
# For this example's sake im gonna be sending mimic data from a client trigger
@socketio.on('trigger_data')
def handle_my_custom_event(json):
    print(f"Received: {json}")
    emit('pos_data', {
        "x": json.get('x'), # int
        "y": json.get('y'), # int
        "heading": json.get('heading'), # int 
    })

# realistically the data here is gonna be sent from the bot.
# For this example's sake im gonna be sending mimic data from a client trigger
@socketio.on('sweep_data')
def handle_my_custom_event(json):
    print(f"Received: {json}")
    emit('sweep_data', {
        "sweepDegree": json.get('sweepDegree'), # int[]
        "sweepHit": json.get('sweepHit')
    })


@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    socketio.run(app, port=5000, debug=True)
