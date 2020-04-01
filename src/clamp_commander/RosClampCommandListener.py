# RosClampCommandListener allows a ClampController to accept commands from ROS
#
# This is intended as a singleton class that provides the functions to
# subscribe and listens to a ROS topic that publish commands
#
# New commands are passed on to a callback function that has to be provided at init.

import datetime
import json
import time

import roslibpy
from roslibpy import Ros


def current_milli_time(): return int(round(time.time() * 1000))


class RosClampCommandListener(Ros):

    def __init__(self, host, command_callback, is_secure=False):
        from twisted.internet import reactor
        reactor.timeout = lambda : 0.00001
        Ros.__init__(self, host, 9090, is_secure)
        self.command_callback = command_callback
        # Setup replier topic.
        self.replier = roslibpy.Topic(self, '/clamp_response', 'std_msgs/String')
        self.replier.advertise()

        def receive_callback(message):
            command_dict = json.loads(message['data'])
            # Reply to the message.
            reply_message = {'msg':'This is just ROS ack. Not necessary clamps moving.' }
            reply_message['sequence_id'] = command_dict['sequence_id']
            reply_message['timestamp'] = current_milli_time()
            self.replier.publish(roslibpy.Message({'data': json.dumps(reply_message)}))
            # Relay message to callback
            self.command_callback(command_dict)

        # Setup listener topic.
        self.listener = roslibpy.Topic(self, '/clamp_command', 'std_msgs/String')
        self.listener.subscribe(receive_callback)

    def reply_ack_result(self, sequence_id, ack_result):
        reply_message = {'msg':' Clamps Ack result.' }
        reply_message['sequence_id'] = sequence_id
        reply_message['timestamp'] = current_milli_time()
        reply_message['ack'] = ack_result
        self.replier.publish(roslibpy.Message({'data': json.dumps(reply_message)}))

# Directly calling this script creates a listener that will print out messages.
# It will also show the one way trip time.


if __name__ == "__main__":

    hostip = '192.168.43.141'

    trip_times = []

    def command_callback(command_dict):
        print(command_dict)
        trip_time = current_milli_time() - int(command_dict['timestamp'])
        trip_times.append(trip_time)
        print("Trip Time = %s" % trip_time)

    client = RosClampCommandListener(hostip, command_callback)
    client.run()

    while (client.is_connected):
        i = input("Press x to quit and see average trip time. \n")
        if i == 'x':
            break

    print("Total %s messages received, trip time = %s" % (len(trip_times), sum(trip_times) / len(trip_times)))

    client.terminate()
