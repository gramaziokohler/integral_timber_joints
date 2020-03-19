from SerialCommander import SerialCommander
from ClampModel import ClampModel

# Create the Commanding Object and monitor the clamps
commander = SerialCommander()
commander.start()
# ClampModel(Address,StepPerMM, BattMin, BattMax)
clamp1 = ClampModel('1', 918, 880, 1024)
clamp2 = ClampModel('2', 918, 880, 1024)
commander.add_clamp(clamp1)
commander.add_clamp(clamp2)


command = input("Please enter a command:\n")
while (command != ""):
    # Send messages
    response1 = commander.message_clamp(clamp1, command, retry=3, time_out_millis=30)
    response2 = commander.message_clamp(clamp2, command, retry=3, time_out_millis=30)
    print("Message sent: %s %s" % (response1 is not None, response2 is not None))
    

    # Monitor status for a while
    for i in range(20):
        commander.update_clamps_status()
        print ("%s, %s" % (clamp1, clamp2))
        
    # Next Command
    command = input("Please enter a command:\n")


print("Serial Port Ends")

