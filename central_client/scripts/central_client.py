import rospy
from deprojection_pipeline_msgs.srv import GetObjectLocations, GetObjectLocationsResponse

class CentralClient:
    def __init__(self) -> None:
        self.get_object_locations_service = rospy.ServiceProxy(
            "get_object_locations",
            GetObjectLocations
        )

    def get_object_locations(self):
        try:
            response = self.get_object_locations_service()
            return response
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")


if __name__ == "__main__":
    rospy.init_node("central_client")
    central_client = CentralClient()
    rospy.sleep(0.1)
    response = central_client.get_object_locations()
    
    # response.result.object_position contains an array of deprojection_pipeline_msgs/ObjectPosition type objects
    orange_position = None
    for object_position in response.result.object_position:
        if object_position.Class == "orange":
            print(f"orange detected at ({object_position.position})")
            orange_position = object_position.position

    # orange_position contains the psoition of the orange object as a PointStamped object