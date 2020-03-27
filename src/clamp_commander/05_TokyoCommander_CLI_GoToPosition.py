from SerialCommanderTokyo import SerialCommanderTokyo
from ClampModel import ClampModel

# Create the Commanding Object and monitor the clamps
commander = SerialCommanderTokyo()
commander.connect_with_CLI()



# Type h to home clamps
user_input = input("Please enter h if you wish to home clamps\n")
if (user_input == "h"):
    result_1 = commander.message_clamp(commander.clamp1,'h', 4)
    result_2 = commander.message_clamp(commander.clamp2,'h', 4)
    print ("Home message Result: %s %s" %(result_1, result_2))

# User controllable position / Becareful homed position is at 93mm 
default_speed = 2.0

# CLI Loop
while (True):
    # Ask for user input and check for termination
    user_input = input("Please enter a jaw position (95 - 225mm) to go:\n")
    if (user_input == ""):
        break

    # Check user Input range
    jaw_target = float(user_input)
    if (jaw_target < 95 or jaw_target > 225): 
        print ("Input out of range.")
        continue

    # Send messages

    send_success1 =  commander.send_clamp_to_jaw_position(commander.clamp1, jaw_target)
    send_success2 =  commander.send_clamp_to_jaw_position(commander.clamp2, jaw_target)
    print("Message sent: %s" % (send_success1))

    # Monitor Status until motor stops
    for i in range(10):
        commander.update_clamp_status(commander.clamp1)
        commander.update_clamp_status(commander.clamp2)
        print ("%s, %s" % (commander.clamp1, commander.clamp2))
        
print("Serial Port Ends")