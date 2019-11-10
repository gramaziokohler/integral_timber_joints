from compas_fab.backends.ros import RosClient
from compas_rrc import *
from compas.geometry import Frame
import time


if __name__ == '__main__':

	ros = RosClient("192.168.163.129")
	ros.run()
	print('ROS connected: %s' % ros.is_connected)

	robot_1 = AbbClient(ros, "/robot_1")
	robot_2 = AbbClient(ros, "/robot_2")

	speed = 500

	### SET TOOL AND WORKOBJECT
	# robot 1
	# set tool
	result = robot_1.send_and_wait(ProjectInstruction('r_A042_SetTool', ['tool0'], feedback_level=1))
	print("Set Tool:", result['instruction'])
	# set workobject
	result = robot_1.send_and_wait(ProjectInstruction('r_A042_SetWobj', ['wobj0'], feedback_level=1))
	print("Set Workobject:", result['instruction'])
	
	# robot 2
	# set tool
	result = robot_2.send_and_wait(ProjectInstruction('r_A042_SetTool', ['tool0'], feedback_level=1))
	print("Set Tool:", result['instruction'])
	# set workobject
	result = robot_2.send_and_wait(ProjectInstruction('r_A042_SetWobj', ['wobj0'], feedback_level=1))
	print("Set Workobject:", result['instruction'])





	### GET CURRENT POSE
	r1 = robot_1.send_and_wait(ProjectInstruction('r_A042_GetRobT', feedback_level=1))
	print("robot 1 pose:", r1['float_values'])
	r2 = robot_2.send_and_wait(ProjectInstruction('r_A042_GetRobT', feedback_level=1))
	print("robot 2 pose:", r2['float_values'])



	### GET CURRENT JOINT CONFIGURATION
	r1 = robot_1.send_and_wait(ProjectInstruction('r_A042_GetJointT', feedback_level=1))
	print("robot 1 configuration:", r1['float_values'])
	r2 = robot_2.send_and_wait(ProjectInstruction('r_A042_GetJointT', feedback_level=1))
	print("robot 2 configuration:", r2['float_values'])


	
	### MOVE J

	r1 = robot_1.send_and_wait(MoveAbsJ([0, 0, 0, 0, 0, 0], [28000, -2000, -4500], speed, Zone.Z50, feedback_level=1))
	r2 = robot_2.send_and_wait(MoveAbsJ([10, 0, 0, 0, 0, 0], [28000, -10000, -4500], speed, Zone.Z50, feedback_level=1))
	print("moveJ r1:", r1)
	print("moveJ r2:", r2)

	print('MoveJ send_and_wait commands sent')


	### STOP TASK FOR ROBOT 1
	robot_1.send(ProjectInstruction('r_A042_Stop'))


	
	r1 = robot_1.send(
		MoveAbsJ([10, 10, 10, 10, 10, 10], [28000, -2000, -4500], speed, Zone.Z50, feedback_level=1))
	r2 = robot_2.send(
		MoveAbsJ([10, 10, 10, 10, 10, 10], [28000, -10000, -4500], speed, Zone.Z50, feedback_level=1))
	print("moveJ r1:", r1)
	print("moveJ r2:", r2)

	print('MoveJ send commands sent')
	

	
	### MOVE L
	r1_frame = Frame.from_quaternion([0.933, 0.067, 0.25, 0.25], point=[30000, 2900, 3000])
	r1_ea = [28000, -2000, -4500]

	r1 = robot_1.send_and_wait(MoveL(r1_frame, r1_ea, speed, Zone.Z50, feedback_level=1))
	print("moveL r1:", r1)



	### STOP TASK FOR ROBOT 2
	robot_2.send(ProjectInstruction('r_A042_Stop'))



	r2_frame = Frame([30000, 9600, 3200], [1,0,0], [0,1,0])
	r2_ea = [28000, -10000, -4500]

	r2 = robot_2.send_and_wait(MoveL(r2_frame, r2_ea, speed, Zone.Z50, feedback_level=1))
	print("moveL r2:", r2)


	print('MoveL send commands sent')


	robot_1.close()
	robot_2.close()

	ros.terminate()

	time.sleep(3)

	print("Robot connection closed")
