#!flask/bin/python
from flask import Flask, request, jsonify
from gpio.GPIOAdaptor import GPIOAdaptor
from datetime import datetime


app = Flask(__name__)


thrust = 1500
heading = 1500
gpio = GPIOAdaptor()
session_commands = []
gpio.set_thrust(thrust)
gpio.set_heading(heading)


@app.route('/')
def index():
    return "Hello, World!"


@app.route('/robot/commands', methods=['POST'])
def post_command():
    print('command thrust:%s heading:%s', request.json)
    thrust = request.json['thrust']
    heading = request.json['heading']
    gpio.set_thrust(thrust)
    gpio.set_heading(heading)
    session_commands.append({
        'thrust':thrust,
        'heading':heading,
        'time_stamp':datetime.now()
    })

    print(request.json)
    return jsonify({}), 201


@app.route('/robot/session', methods=['POST'])
def start_session():
    session_commands = []


@app.route('/robot/session', methods=['GET'])
def end_session():
    return jsonify(session_commands)


if __name__ == '__main__':
    app.run(debug=True)
