# TokyoClampCommunicationViaRos provides simple to use functions to send clamp commands via ROS
# The receiving Clamp Controller should be started.

# Typical usage:
#   hostip = '192.168.43.141'
#   clamps_connection = TokyoClampCommunicationViaRos(hostip)
# Command to send clamp to target (non-blocking)
#   clamps_connection.send_ROS_VEL_GOTO_COMMAND(100.0, 1.0)
# Command to send clamp to target (blocking)
#   success = clamps_connection.send_ROS_VEL_GOTO_COMMAND_wait(100.0, 1.0, 1000)
# Command to stop clamps (non-blocking)
#   clamps_connection.send_ROS_STOP_COMMAND(['1','2'])

# Directly running this file creates a CLI for sending movement commands.

import time, datetime
import json 
import roslibpy
current_milli_time = lambda: int(round(time.time() * 1000))

from roslibpy import Ros

class TokyoClampCommunicationViaRos(Ros):
        
    def __init__(self, host_ip:str):
        Ros.__init__(self, host=host_ip, port=9090)
        self.run()

        # Communication control
        self.sequence_id = -1

        self.sent_messages = {}
        self.trip_times = []
        
        def callback(message_string):
            receive_time = current_milli_time()
            # Retrive the sent message and compare time
            feedback_message = json.loads(message_string['data'])
            sequence_id = feedback_message['sequence_id']
            org_message = self.sent_messages[sequence_id]
            org_message['received'] = True
            send_time = int(org_message['timestamp'])
            bounce_time = int(feedback_message['timestamp'])
    
            # Print it to UI and keep track of one way latency.
            rtt = receive_time - send_time
            t1t = bounce_time - send_time
            t2t = receive_time - bounce_time
            # print ("Received; %s" % feedback_message)
            print ("Received Message %s, RoundTripTime = %s (%s + %s)" % (sequence_id, rtt, t1t, t2t))
            self.trip_times.append(rtt)

        # Setup talker to send message
        self.talker = roslibpy.Topic(self, '/clamp_command', 'std_msgs/String')
        self.talker.advertise()

        # Setup listener to listen for feedback
        self.listener = roslibpy.Topic(self, '/clamp_response', 'std_msgs/String')
        self.listener.subscribe(callback)

        # Send an initial message Sequence id will be -1
        self.send_ros_command("ROS_NEW_SENDER_INITIALIZE", "")

    # Returns the sequence_id of sent message
    def send_ros_command(self, instruction_type: str, instruction_body: str):
        # Create message to send
        message = {}
        message['sequence_id'] = self.sequence_id
        message['timestamp'] = current_milli_time()
        message['instruction_type'] = instruction_type
        message['instruction_body'] = instruction_body
        self.talker.publish(roslibpy.Message({'data' : json.dumps(message)}))
        # Keep track of sent message
        message['received'] = False # Keep track of ACK.
        self.sent_messages[self.sequence_id] = message
        self.sequence_id += 1
        print("Sent Message to \clamp_instruction:", message)
        return self.sequence_id - 1
    
    # Return true if sending is successful. False if timeout
    def send_ROS_VEL_GOTO_COMMAND_wait(self, clamps_id: str, position: float, velocity: float, timeout_ms: int = 1000) -> bool:
        start_time = current_milli_time()
        sequence_id = self.send_ROS_VEL_GOTO_COMMAND(clamps_id, position, velocity)
        while (current_milli_time() - start_time < timeout_ms):
            if self.sent_messages[sequence_id] == True:
                return True
        
        return False

    # Returns the sequence_id of sent message
    def send_ROS_VEL_GOTO_COMMAND(self, clamps_id: str, position: float, velocity: float) -> int:
        instructions = []
        for clamp_id in clamps_id:
            instructions.append((clamp_id, position, velocity))
        return self.send_ros_command("ROS_VEL_GOTO_COMMAND",instructions)
         
    # Returns the sequence_id of sent message
    def send_ROS_STOP_COMMAND(self, clamps_id: str):
        instructions = []
        for clamp_id in clamps_id:
            instructions.append((clamp_id))
        return self.send_ros_command("ROS_STOP_COMMAND",instructions)

    def terminate(self):
        self.talker.unadvertise()
        Ros.terminate(self)

# CLI Loop
if __name__ == "__main__":

    hostip = '192.168.43.141'
    clamps_connection = TokyoClampCommunicationViaRos(hostip)

    # Command to send clamp to target (non-blocking)
    # clamps_connection.send_ROS_VEL_GOTO_COMMAND(100.0, 1.0)

    # Command to send clamp to target (blocking)
    # success = clamps_connection.send_ROS_VEL_GOTO_COMMAND_wait(100.0, 1.0, 1000)


    while clamps_connection.is_connected:
        i = input("Type a position (95-220) , velocity  (0.1 - 3.0) (to Send a test Message topic=/clamp_command , x to quit.\n")
        # Function to Stop UI 
        if i == 'x':
            break
        if i == 's':
            # Stop
            clamps_connection.send_ROS_STOP_COMMAND(['1','2'])
            continue
        else:
            # Goto Pos,Vel
            try:
                _pos, _vel = i.split(',')
                position = float(_pos)
                velocity = float(_vel)
                if (position < 95.0 or position > 220.0):
                    raise  ValueError
            except:
                print ("Bad Input, position range (95-220). Try again:\n")
                continue
            clamps_connection.send_ROS_VEL_GOTO_COMMAND_wait(['1','2'],position, velocity)
        
        # place message in dict to allow later retrivel for time comparision.

    clamps_connection.terminate()

