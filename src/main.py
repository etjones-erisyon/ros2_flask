#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import threading

from flask import Flask, render_template, render_template_string, redirect
from std_msgs.msg import String
from lyra_types.msg import DioInPinState, DioOutPinState
import html


DATA_STORE = {}

app = Flask(__name__)

# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.

def main(args=None):
    rclpy.init(args=args)

    minimal_listener = MinimalListener()

    rclpy.spin(minimal_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_listener.destroy_node()
    rclpy.shutdown()



class MinimalListener(Node):
    def __init__(self):
        super().__init__("minimal_listener")
        self.subscription = self.create_subscription(DioInPinState, '/dio/in', self.dio_in_callback, 10)


    def dio_in_callback(self, msg:DioInPinState):
        global DATA_STORE
        DATA_STORE['/dio/in'] = msg



threading.Thread(target=main).start()
# threading.Thread(target=lambda: rclpy.init_node('test_node', disable_signals=True)).start()
# pubMotion = rclpy.Publisher('/requestMotion01', String, queue_size=1)
# pubStop = rclpy.Publisher('/requestStop01', String, queue_size=1)

# sub_dio = rclpy.Subscriber('/dio/in', DioInPinState, dio_in_callback)
# NGROK = rclpy.get_param('/ngrok', None)



@app.route('/dio/in')
def show_dio_in():
    global DATA_STORE
    msg = DATA_STORE.get('/dio/in', (None, None))
    # return html.dio_in(DATA_STORE.get('/dio/in', None))
    return render_template_string(f'''
    <div id="dio-in" class="dio">
        <span>{msg.pin = }</span><br>
        <span>{msg.state = }</span><br>                       
    </div>
    ''')

@app.route('/')
def default():
    return redirect('/info')


@app.route('/info')
def info():
    return html.info()


@app.route('/send_movement_command/<direction>', methods = ['GET'])
def send_movement_command(direction):
    if any(direction in d for d in ['forward','backward','left','right', 'stop']):
        # new ROSLIB.Message({data: motion})
        if (direction == 'stop'):
            pubStop.publish( direction.upper() )
        else:
            pubMotion.publish( direction.upper() )
            return html.success(direction)
    else:
        msg = 'Direction not recognized'
        return html.failure(msg)


if __name__ == '__main__':
    # from og server
	app.run(host='0.0.0.0', debug=True)
