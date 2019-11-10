from compas_fab.backends.ros import RosClient
from compas_rrc import *
import time






o = 10
re = list(range(o))
re.reverse()

configs = []
for g in range(30):
	for c in range(o):
		j = [c]*6
		configs.append(j)
	for k in re:
		j = [k]*6
		configs.append(j)


#for c in configs:
#	print(c)



ros = RosClient("192.168.163.129")
print('ROS connected: %s' % ros.is_connected)

robot_1 = AbbClient(ros, "/robot_1")
robot_2 = AbbClient(ros, "/robot_2")
robot_1.run()
robot_2.run()

print('Robot 11 connected: %s' % robot_1.ros.is_connected)
print('Robot 12 connected: %s' % robot_2.ros.is_connected)




speed = 500
r1_ea = [18000, -2000, -4500]

for c in configs:
	r1 = robot_1.send_and_wait( MoveAbsJ(c, r1_ea, speed, Zone.Z50, feedback_level=1) )


robot_1.close()
robot_1.terminate()

print("Robot connection closed")
