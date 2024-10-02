import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PointStamped, Pose
from tf.transformations import quaternion_from_euler, quaternion_multiply
from std_srvs.srv import SetBool
from motion_planning_server_msgs.msg import PickPlaceAction, PickPlaceActionGoal, PickPlaceActionResult
from motion_planning_server_msgs.msg import MovePreactionAction, MovePreactionActionGoal, MovePreactionActionResult
import actionlib


# MACRO FOR TOOL HEIGHT 
TOOL_HEIGHT = 0.12
# MACRO FOR X,Y AND Z OFFSET
X_OFFSET = 0.0
Y_OFFSET = 0.0
Z_OFFSET = 0.0

class Motion_planner:

    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.loginfo("Initializing motion planner")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.gripper_client = rospy.ServiceProxy("gripper", SetBool)

        self.pre_action_joint_state = [0, -tau/8, 0, -tau/4, 0, tau/6, 0]
        self.waypoints = []

        # create action server for pick and place
        self.pick_place_server = actionlib.SimpleActionServer(
            "pick_place", PickPlaceAction, self.pick_place_callback, auto_start=False
        )

        self.pick_place_server.start()



    def pick_place_callback(self, goal:PickPlaceActionGoal):
        rospy.loginfo("Received pick and place goal")
        start = goal.source
        end = goal.destination
        
        success = self.pick_and_place(start, end)
        
        # set goal to success
        result = PickPlaceActionResult()
        result.result = success

        if success:
            self.pick_place_server.set_succeeded(result)
        else:
            self.pick_place_server.set_aborted(result)


    def pick_and_place(self,start:PointStamped, end:PointStamped):
        rospy.loginfo("Started pick and place with start : %s and end : %s", start, end)
        
        orientation = Pose()
        orientation.orientation.x = -0.5825436479626629
        orientation.orientation.y = -0.8127928845856666
        orientation.orientation.z = 0.0029525975807408256
        orientation.orientation.w = 0.001380997027855892

        # plan a cartesian path to pick, prepick -> pick
        waypoints = []
        prepick = Pose()
        prepick.position = start.point
        prepick.orientation = orientation.orientation
        prepick.position.z += 0.1 + TOOL_HEIGHT# move up 10 cm
        waypoints.append(copy.deepcopy(prepick))
        
        pick = copy.deepcopy(prepick)
        pick.position.z -= 0.1 # move down 10 cm
        pick.orientation = orientation.orientation
        waypoints.append(copy.deepcopy(pick))

        # rospy.loginfo("Waypoints : %s", waypoints)
        # plan a cartesian path
        try : 
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0,  # jump_threshold
            )
        except Exception as e:
            print(e)
            return False

        # display the plan
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # execute the plan
        rospy.loginfo("Executing prepick -> pick plan")
        try : 
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
        except Exception as e:
            print(e)
            return False

        # activate the gripper here
        rospy.loginfo("Activating gripper")
        self.gripper_client.call(True)

        rospy.sleep(0.3)

        # oscillate up and down 2 times with an amplitude of 2cm
        waypoints = []
        oscillate_low = copy.deepcopy(prepick)
        oscillate_low.position.z += 0.005
        oscillate_high = copy.deepcopy(prepick)
        oscillate_high.position.z += 0.04
        waypoints.append(copy.deepcopy(prepick))
        waypoints.append(copy.deepcopy(oscillate_high))
        waypoints.append(copy.deepcopy(oscillate_low))
        waypoints.append(copy.deepcopy(prepick))
        # waypoints.append(copy.deepcopy(oscillate_low))

        # plan a cartesian path
        try : 
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0,  # jump_threshold
            )
        except Exception as e:
            print(e)
            return False

        # display the plan
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # execute the plan
        rospy.loginfo("Executing oscillation")
        try : 
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
        except Exception as e:
            print(e)
            return False

        # plan cartesian path to prepick -> place
        waypoints = []
        waypoints.append(copy.deepcopy(prepick))
        preplace = Pose()
        preplace.position = end.point
        preplace.orientation = orientation.orientation
        preplace.position.z += 0.15 + TOOL_HEIGHT
        waypoints.append(copy.deepcopy(preplace))
        place = copy.deepcopy(preplace)
        place.position.z -= 0.09
        waypoints.append(copy.deepcopy(place))

        # plan a cartesian path
        try : 
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0,  # jump_threshold
            )
        except Exception as e:
            print(e)
            return False

        # display the plan
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # execute the plan
        rospy.loginfo("Executing prepick -> place plan")
        try : 
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
        except Exception as e:
            print(e)
            return False

        # deactivate the gripper here
        waypoints = []
        rospy.loginfo("Deactivating gripper")
        self.gripper_client.call(False)

        rospy.sleep(0.3)

        # plan cartesian path to place -> preplace
        waypoints = []
        waypoints.append(copy.deepcopy(preplace))

        # plan a cartesian path
        try : 
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0,  # jump_threshold
            )
        except Exception as e:
            print(e)
            return False

        # display the plan
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # execute the plan
        rospy.loginfo("Executing place -> preplace plan")
        try : 
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
        except Exception as e:
            print(e)
            return False

        return True
        
if __name__  == "__main__":
    rospy.init_node("pick_place_server", anonymous=True)
    mp = Motion_planner()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")
    sys.exit(0)
        


