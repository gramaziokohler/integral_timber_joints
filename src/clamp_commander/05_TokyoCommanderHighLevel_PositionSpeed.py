from SerialCommander import SerialCommander
from ClampModel import ClampModel

# Create the Commanding Object and monitor the clamps
commander = SerialCommander()
commander.start()
# ClampModel(Address,StepPerMM, JawOffset, SoftLimitMin_mm, SoftLimitMax_mm,  BattMin, BattMax)
clamp1 = ClampModel('1', 918, 95.0, 94.0, 225.0, 880.0, 1004.0)
clamp2 = ClampModel('2', 918, 95.0, 94.0, 225.0, 880.0, 1004.0)
commander.add_clamp(clamp1)
commander.add_clamp(clamp2)

# Get status
commander.update_clamps_status(3)
for addr , clamp in commander.clamps.items():
    print (clamp)

def press_continue_or_stop(msg):
    command = input(msg)
    if command.startswith("s"): 
        commander.stop_clamps()
        raise SystemExit(0)

# If Clamps are homed already, skip the home sequence
if (not clamp1.ishomed) or (not clamp2.ishomed):
    press_continue_or_stop("Press enter to continue: Home Clamps")
    response1 = commander.message_clamp(clamp1, "h", retry=3, time_out_millis=30)
    response2 = commander.message_clamp(clamp2, "h", retry=3, time_out_millis=30)
    print("Message success: %s %s" % (response1 is not None, response2 is not None))


    press_continue_or_stop("Press enter to continue: Set Fast Velocity to 3mm/s (v2754)")
    response1 = commander.message_clamp(clamp1, "v2754", retry=3, time_out_millis=30)
    response2 = commander.message_clamp(clamp2, "v2754", retry=3, time_out_millis=30)
    print("Message success: %s %s" % (response1 is not None, response2 is not None))

    press_continue_or_stop("Press enter to continue: Move Clamps to Zero")
    response1 = commander.message_clamp(clamp1, "g0", retry=3, time_out_millis=30)
    response2 = commander.message_clamp(clamp2, "g0", retry=3, time_out_millis=30)
    print("Message success: %s %s" % (response1 is not None, response2 is not None))
else:
    press_continue_or_stop("Press enter to continue: Set Fast Velocity to 3mm/s (v2754)")
    response1 = commander.message_clamp(clamp1, "v2754", retry=3, time_out_millis=30)
    response2 = commander.message_clamp(clamp2, "v2754", retry=3, time_out_millis=30)
    print("Message success: %s %s" % (response1 is not None, response2 is not None))

press_continue_or_stop("Press enter to continue: Move Clamps to Open Jaw (210 mm Jaw) (g105570)")
response1 = commander.message_clamp(clamp1, "g105570", retry=3, time_out_millis=30)
response2 = commander.message_clamp(clamp2, "g105570", retry=3, time_out_millis=30)
print("Message success: %s %s" % (response1 is not None, response2 is not None))

press_continue_or_stop("Press enter to continue: Move Clamps to Engage Position (165 mm Jaw) (g64260)")
response1 = commander.message_clamp(clamp1, "g64260", retry=3, time_out_millis=30)
response2 = commander.message_clamp(clamp2, "g64260", retry=3, time_out_millis=30)
print("Message success: %s %s" % (response1 is not None, response2 is not None))

press_continue_or_stop("Press enter to continue: Set Slow Velocity to 2mm/s (v1836)")
response1 = commander.message_clamp(clamp1, "v1836", retry=3, time_out_millis=30)
response2 = commander.message_clamp(clamp2, "v1836", retry=3, time_out_millis=30)
print("Message success: %s %s" % (response1 is not None, response2 is not None))

# command = input("Press enter to continue: Close Clamps to Closed Position (110 mm Jaw) (g13770)")
# response1 = commander.message_clamp(clamp1, "g13770", retry=3, time_out_millis=30)
# response2 = commander.message_clamp(clamp2, "g13770", retry=3, time_out_millis=30)
# print("Message success: %s %s" % (response1 is not None, response2 is not None))

press_continue_or_stop("Press enter to continue: Close Clamps to Closed Position (100 mm Jaw) (g4590)")
response1 = commander.message_clamp(clamp1, "g4590", retry=3, time_out_millis=30)
response2 = commander.message_clamp(clamp2, "g4590", retry=3, time_out_millis=30)
print("Message success: %s %s" % (response1 is not None, response2 is not None))

press_continue_or_stop("Press enter to continue: Set Fast Velocity to 3mm/s (v2754)")
response1 = commander.message_clamp(clamp1, "v2754", retry=3, time_out_millis=30)
response2 = commander.message_clamp(clamp2, "v2754", retry=3, time_out_millis=30)
print("Message success: %s %s" % (response1 is not None, response2 is not None))

press_continue_or_stop("Press enter to continue: Release Clamps to Open Jaw (210 mm Jaw) (g105570)")
response1 = commander.message_clamp(clamp1, "g105570", retry=3, time_out_millis=30)
response2 = commander.message_clamp(clamp2, "g105570", retry=3, time_out_millis=30)
print("Message success: %s %s" % (response1 is not None, response2 is not None))

# Print Clamp Latest Status
for addr , clamp in commander.clamps.items():
    print (clamp)

commander.serial_port.close()
print("Script End / Serial Port Closed")
