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

        # create an action server for move preaction
        self.move_preaction_server = actionlib.SimpleActionServer(
            "move_preaction", MovePreactionAction, self.move_preaction_callback, auto_start=False
        )

        self.pick_place_server.start()

    def move_preaction_callback(self, goal:MovePreactionActionGoal):
        rospy.loginfo("Received move preaction goal")
        success = self.move_to_preaction()
        result = MovePreactionActionResult()
        result.result = success

        if success:
            self.move_preaction_server.set_succeeded(result)
        else:
            self.move_preaction_server.set_aborted(result)


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
        # plan a cartesian path to pick, prepick -> pick
        waypoints = []
        prepick = Pose()
        prepick.position = start.point
        prepick.orientation.w = 1.0
        prepick.position.z += 0.1 # move up 10 cm
        waypoints.append(copy.deepcopy(prepick))
        
        pick = prepick
        pick.position.z -= 0.1 # move down 10 cm
        pick.orientation.w = 1
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
        waypoints = []

        # activate the gripper here
        rospy.loginfo("Activating gripper")
        gripper_state = SetBool()
        gripper_state.data = True
        # self.gripper_client.call(gripper_state)

        rospy.sleep(0.3)

        # plan cartesian path to prepick -> place
        waypoints.append(prepick)
        preplace = Pose()
        preplace.position = end.point
        preplace.orientation.w = 1.0
        preplace.position.z += 0.2
        waypoints.append(preplace)

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
        waypoints = []

        # deactivate the gripper here
        rospy.loginfo("Deactivating gripper")
        gripper_state.data = False
        # self.gripper_client.call(gripper_state)

        return True
    
    def move_to_preaction(self):
        joint_goal = self.pre_action_joint_state
        try : 
            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()
        except Exception as e:
            print(e)
            return False
        return True
        
if __name__  == "__main__":
    rospy.init_node("mpserver", anonymous=True)
    mp = Motion_planner()

    # # modify xyz values to suitable readings
    # start = PointStamped()
    # start.point.x = 0.4
    # start.point.y = 0.1
    # start.point.z = 0.4
    # end = PointStamped()
    # end.point.x = 0.4
    # end.point.y = -0.1
    # end.point.z = 0.4

    rospy.spin()
    # rospy.sleep(0.1)
    # mp.move_to_preaction()
    # rospy.sleep(0.1)
    # mp.pick_and_place(start,end)
    # rospy.sleep(0.1)
    # mp.move_to_preaction()

    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")
    sys.exit(0)
        


