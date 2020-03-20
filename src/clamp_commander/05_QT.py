from SerialCommanderTokyo import SerialCommanderTokyo
from ClampModel import ClampModel

# Create the Commanding Object and monitor the clamps
commander = SerialCommanderTokyo()
commander.start()


jaw_target = input("Please enter a jaw position (95 - 225mm) to go:\n")
default_speed = 2.0
while (jaw_target != ""):
    jaw_target = float(jaw_target)
    # Send messages
    send_success1 =  commander.send_all_clamps_to_jaw_position(jaw_target, default_speed)
    print("Message sent: %s" % (send_success1))

    # Monitor Status until motor stops
    for i in range(10):
        commander.update_clamps_status()
        print ("%s, %s" % (commander.clamp1, commander.clamp2))
        
    # Next Command
    jaw_target = input("Please enter a jaw position (95 - 225mm) to go:\n")

print("Serial Port Ends")