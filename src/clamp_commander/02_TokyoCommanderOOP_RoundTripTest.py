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

#Before the update_clamps_status() call.
print (clamp1)
print (clamp2)

# Test them with difference allowable timeout values
# Result is that starting from 16ms, there is a response.  
for i in range(10,30,1):
    print ("- Testing Allowable Timeout: %s"%(i))
    commander.update_clamps_status(i)
    import time
    time.sleep(0.1)

# After the update_clamps_status() call.
print (clamp1)
print (clamp2)

