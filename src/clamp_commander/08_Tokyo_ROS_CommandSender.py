import time
from TokyoClampCommunicationViaRos import TokyoClampCommunicationViaRos

hostip = '192.168.0.117'
clamps_connection = TokyoClampCommunicationViaRos(hostip)

time.sleep(0.5)
clamps_connection.send_ros_command("ROS_NEW_SENDER_INITIALIZE", "Resend1")
clamps_connection.send_ros_command("ROS_NEW_SENDER_INITIALIZE", "Resend2")
time.sleep(0.5)
velocity_approach = 4.0
velocity_closure = 2.0
joint_thickness_1 = 136 #Top Clamp
joint_thickness_2 = 111  #Bottom Clamp
approach_offset = 50

# Instruction 1 is to reach approach plane
while (True):
    instructions = []
    instructions.append(('1', joint_thickness_1 + approach_offset, velocity_approach))   #Top Clamp
    instructions.append(('2', joint_thickness_2 + approach_offset, velocity_approach))    #Bottom Clamp
    i = input(" Next Command (1): %s\nPress Enter to continue:" % instructions)
    clamps_connection.send_ros_command("ROS_VEL_GOTO_COMMAND",instructions)

    # Instruction 2 is to reach final plane

    instructions = []
    instructions.append(('1', joint_thickness_1, velocity_closure))     #Top Clamp
    instructions.append(('2', joint_thickness_2, velocity_closure))     #Bottom Clamp
    i = input(" Next Command (2): %s\nPress Enter to continue:" % instructions)
    clamps_connection.send_ros_command("ROS_VEL_GOTO_COMMAND",instructions)

    # Instruction 3 is to open clamp

    instructions = []
    instructions.append(('1', 220, 5))     #Top Clamp
    instructions.append(('2', 220, 5))     #Bottom Clamp
    i = input(" Next Command (3): %s\nPress Enter to continue:" % instructions)
    clamps_connection.send_ros_command("ROS_VEL_GOTO_COMMAND",instructions)


# Command to send clamp to target (blocking)
#   success = clamps_connection.send_ROS_VEL_GOTO_COMMAND_wait(100.0, 1.0, 1000)
# Command to stop clamps (non-blocking)
#   clamps_connection.send_ROS_STOP_COMMAND(['1','2'])