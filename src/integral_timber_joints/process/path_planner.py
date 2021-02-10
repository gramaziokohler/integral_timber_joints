import math
import time

from compas.geometry import Frame
from compas_fab.backends import RosClient
from compas_fab.robots import CollisionMesh, Configuration, PlanningScene


class PathPlanner:
    """The PathPlanner Class stores objects, states and connections related to
    performing moveit enquery from ROS backend.
    Includes static IK solution, linear and freespace trajectory planning.
    """

    def __init__(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        self.process = process  # type: RobotClampAssemblyProcess
        self.ros_client = None  # type: RosClient
        self.robot = None  # type: Robot
        self.scene = None  # type: PlanningScene

        # Variable for keeping last planned state
        self.last_trajectory = None  # type: compas_fab.robots.JointTrajectory
        self.last_planning_success = False  # type: bool

    def connect_to_ros_planner(self, ip_address):
        """ Connects to a pre-started ROS backend via compas_fab

        Example ROS start
        -----------------
        roslaunch rosbridge_server rosbridge_websocket.launch
        roslaunch file_server file_server.launch
        roslaunch ur5_moveit_config demo.launch (Depends on Robot)
        roslaunch rfl_moveit_config demo.launch (Depends on Robot)
        """
        # Terminate previous RosClient
        self.disconnect_to_ros_planner()

        # Connect to RosClient with a timeout wait.
        self.ros_client = RosClient(host=ip_address)  # type: RosClient
        self.ros_client.run()
        self.ros_client.connect()

        # Load Robot without geometry (faaster)
        self.robot = self.ros_client.load_robot(load_geometry=False)
        self.scene = PlanningScene(self.robot)

    def disconnect_to_ros_planner(self):
        if self.ros_client is not None:
            self.ros_client.terminate()

    def append_collision_mesh(self, mesh, id):
        # type: (compas.datastructure.Mesh, str) -> None
        cm = CollisionMesh(mesh, id)
        self.scene.append_collision_mesh(cm, scale=True)
        time.sleep(1)

    def remove_collision_mesh(self, id):
        # type: (str) -> None
        self.scene.remove_collision_mesh(id)


class RFLPathPlanner(PathPlanner):
    """RFL Robot Specific Path Planner
    Units in millimeters

    The backend should be started with:
    roslaunch rosbridge_server rosbridge_websocket.launch
    roslaunch file_server file_server.launch
    roslaunch rfl_moveit_config demo.launch (Depends on Robot)
    """

    def __init__(self, process):
        # type: (RobotClampAssemblyProcess) -> None
        PathPlanner.__init__(self, process)
        self.current_configuration = None  # type: Configuration

        self.free_motion_tolerance_position_mm = 0.1
        self.free_motion_tolerance_angular_deg = 0.5
        self.free_motion_allowed_planning_time = 5.0
        self.free_motion_planner_id = "RRT"
        self.linear_motion_step_distance = 1.0
        self.planning_group = "robot11_eaXYZ"

    def connect_to_ros_planner(self, ip_address):
        PathPlanner.connect_to_ros_planner(self, ip_address)
        self.robot.scale(1000)
        assert self.robot.name

    @classmethod
    def rfl_world_start_configuration(cls):
        # type: () -> Configuration
        return Configuration(
            values=(
                0,
                0, -4915,
                0, 0, 0, 0, 0, 0,
                -12237, -4915,
                0, 0, 0, 0, 0, 0,
                38000,
                0, -4915,
                0, 0, 0, 0, 0, 0,
                -12237, -4915,
                0, 0, 0, 0, 0, 0
            ),
            types=(
                2,
                2, 2,
                0, 0, 0, 0, 0, 0,
                2, 2,
                0, 0, 0, 0, 0, 0,
                2,
                2, 2,
                0, 0, 0, 0, 0, 0,
                2, 2,
                0, 0, 0, 0, 0, 0
            ),
            joint_names=(
                'bridge1_joint_EA_X',
                'robot11_joint_EA_Y', 'robot11_joint_EA_Z',
                'robot11_joint_1', 'robot11_joint_2', 'robot11_joint_3', 'robot11_joint_4', 'robot11_joint_5', 'robot11_joint_6',
                'robot12_joint_EA_Y', 'robot12_joint_EA_Z',
                'robot12_joint_1', 'robot12_joint_2', 'robot12_joint_3', 'robot12_joint_4', 'robot12_joint_5', 'robot12_joint_6',
                'bridge2_joint_EA_X',
                'robot21_joint_EA_Y', 'robot21_joint_EA_Z',
                'robot21_joint_1', 'robot21_joint_2', 'robot21_joint_3', 'robot21_joint_4', 'robot21_joint_5', 'robot21_joint_6',
                'robot22_joint_EA_Y', 'robot22_joint_EA_Z',
                'robot22_joint_1', 'robot22_joint_2', 'robot22_joint_3', 'robot22_joint_4', 'robot22_joint_5', 'robot22_joint_6'
            )
        )

    @classmethod
    def rfl_timber_start_configuration(cls):
        # type: () -> Configuration
        return Configuration(
            values=(
                18700,
                0, -4900,
                0, 0, 0, 0, 0, 0,
                -11000, -4900,
                0, 0, 0, 0, 0, 0,
                35000,
                0, -4900,
                0, 0, 0, 0, 0, 0,
                -11000, -4900,
                0, 0, 0, 0, 0, 0
            ),
            types=(
                2,
                2, 2,
                0, 0, 0, 0, 0, 0,
                2, 2,
                0, 0, 0, 0, 0, 0,
                2,
                2, 2,
                0, 0, 0, 0, 0, 0,
                2, 2,
                0, 0, 0, 0, 0, 0
            ),
            joint_names=(
                'bridge1_joint_EA_X',
                'robot11_joint_EA_Y', 'robot11_joint_EA_Z',
                'robot11_joint_1', 'robot11_joint_2', 'robot11_joint_3', 'robot11_joint_4', 'robot11_joint_5', 'robot11_joint_6',
                'robot12_joint_EA_Y', 'robot12_joint_EA_Z',
                'robot12_joint_1', 'robot12_joint_2', 'robot12_joint_3', 'robot12_joint_4', 'robot12_joint_5', 'robot12_joint_6',
                'bridge2_joint_EA_X',
                'robot21_joint_EA_Y', 'robot21_joint_EA_Z',
                'robot21_joint_1', 'robot21_joint_2', 'robot21_joint_3', 'robot21_joint_4', 'robot21_joint_5', 'robot21_joint_6',
                'robot22_joint_EA_Y', 'robot22_joint_EA_Z',
                'robot22_joint_1', 'robot22_joint_2', 'robot22_joint_3', 'robot22_joint_4', 'robot22_joint_5', 'robot22_joint_6'
            )
        )

    @classmethod
    def rfl_timber_test_target(cls):
        # type: () -> Frame
        return Frame([19823.254, 6008.556, 922.020],
                     [-1.0, 0.0, 0.0],
                     [0.0, 1.0, 0.0])

    def test_plan_motion(self):
        # type: () -> None
        """ Test planning a free space motion with ros
        connect_to_ros_planner() should be run before calling this function.
        """

        # start configuration
        start_configuration = self.rfl_timber_start_configuration()

        # goal_constraints parameters
        frame_WCF = self.rfl_timber_test_target()
        tolerance_position = 1  # 1mm
        tolerance_xaxis_deg, tolerance_yaxis_deg, tolerance_zaxis_deg = (1, 1, 1)  # degrees
        tolerances_axes = [math.radians(tolerance_xaxis_deg), math.radians(tolerance_yaxis_deg), math.radians(tolerance_zaxis_deg)]
        group = "robot11_eaXYZ"
        goal_constraints = self.robot.constraints_from_frame(frame_WCF, tolerance_position, tolerances_axes, group)

        # path constraint
        path_constraints = []

        # Path planning in a try block
        try:
            trajectory = self.robot.plan_motion(goal_constraints,
                                                start_configuration,
                                                group,
                                                allowed_planning_time=2.0,
                                                planner_id="RRT",
                                                path_constraints=path_constraints)  # type: compas_fab.robots.JointTrajectory
            print("Computed kinematic path with %d configurations." % len(trajectory.points))
            print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)
            if (trajectory.fraction == 1):
                self.last_planning_success = True
            print("%3.0f%% of the path is computed." % (trajectory.fraction * 100))

            self.last_trajectory = trajectory

        except Exception as e:
            print("robot.plan_motion error:", e)
            self.last_planning_success = False

    def plan_motion(self, target_frame, free_motion=True, start_configuration=None, verbose=False):
        # type: ()-> # type: compas_fab.robots.JointTrajectory

        # Sanity Check
        assert self.ros_client.is_connected

        # Start configuration
        if start_configuration is None:
            start_configuration = self.current_configuration
        if start_configuration is None:
            start_configuration = self.rfl_timber_start_configuration()

        # path constraint
        path_constraints = []

        # Path planning in a try block

        if free_motion:
            # goal_constraints parameters
            tolerances_axes = [math.radians(self.free_motion_tolerance_angular_deg)] * 3
            goal_constraints = self.robot.constraints_from_frame(target_frame, self.free_motion_tolerance_position_mm, tolerances_axes, self.planning_group)
            try:
                trajectory = self.robot.plan_motion(
                    goal_constraints=goal_constraints,
                    start_configuration=start_configuration,
                    group=self.planning_group,
                    options={
                        'allowed_planning_time': self.free_motion_allowed_planning_time,
                        'planner_id': self.free_motion_planner_id,
                        'path_constraints': path_constraints
                    }
                )
            except Exception as e:
                if verbose:
                    print("robot.plan_motion error:", e)
                    # raise e
                self.last_planning_success = False
                return None
        else:
            try:
                trajectory = self.robot.plan_cartesian_motion([target_frame],
                                                              start_configuration,
                                                              max_step=self.linear_motion_step_distance,
                                                              avoid_collisions=True
                                                              )
            except Exception as e:
                if verbose:
                    print("robot.plan_cartesian_motion error:", e)
                    # raise e
                self.last_planning_success = False
                return None
        if verbose:
            print("Computed kinematic path with %d configurations." % len(trajectory.points))
        if verbose:
            print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)
        if (trajectory.fraction == 1):
            self.last_planning_success = True
        if verbose:
            print("%3.0f%% of the path is computed." % (trajectory.fraction * 100))

        self.last_trajectory = trajectory
        return trajectory

    def ik_solution(self, target_frame, start_configuration=None, verbose=False):
        # Start configuration
        if start_configuration is None:
            start_configuration = self.current_configuration
        if start_configuration is None:
            start_configuration = self.rfl_timber_start_configuration()

        try:
            configuration = self.robot.inverse_kinematics(
                target_frame,
                start_configuration=start_configuration,
                group=self.planning_group,
                options={'return_full_configuration': True})
            print("robot.ik_solution success: %s " % configuration)
            return configuration
        except Exception as e:
            if verbose:
                print("robot.ik_solution error:", e)
                # raise e
            return None


if __name__ == "__main__":

    pp = RFLPathPlanner(None)
    pp.connect_to_ros_planner("192.168.43.28")
    print('Connected: %s' % pp.ros_client.is_connected)
    print('Loaded Robot: %s' % pp.robot.name)
    pp.test_plan_motion()

    # Test Print out the trajectory
    print(pp.last_trajectory.joint_names)
    for point in pp.last_trajectory.points:
        print("t=%.3f: %s" % (point.time_from_start.seconds, point.values))
    pp.disconnect_to_ros_planner()
