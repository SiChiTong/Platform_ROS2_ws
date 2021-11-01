map_keyword = "yaml_filename"
from nav2_commander import *


class test_navigator(platformNavigator):
    def set_map(self, map, ratio_x=1.0, ratio_y=1.0):
        self.map = ProcessedMap(map, ratio_x=ratio_x, ratio_y=ratio_y)

class ProcessedMap_test(ProcessedMap):
    def __init__(self, map, ratio_x=1.0, ratio_y=1.0):
        if type(map) == OccupancyGrid:
            size_x = int(map.info.width)
            size_y = int(map.info.height)
            resolution = map.info.resolution
            origin = map.info.origin
            np_map : np.ndarray = np.array(map.data).reshape([size_y, size_x]).astype(np.uint8)
            np_map[np.isin(np_map, [100])] = 255
            np_map = np.flip(np_map,0)

        elif type(map) == Costmap:
            size_x = map.metadata.size_x
            size_y = map.metadata.size_y
            resolution = map.metadata.resolution
            origin = map.metadata.origin
            np_map = np.array(map.data).reshape([size_y, size_x])
            np_map = np.flip(np_map,0)

        elif type(map) == dict:
            np_map = cv2.imread(map['image'])
            np_map = cv2.cvtColor(np_map,cv2.COLOR_BGR2GRAY)
            size_x = np_map.shape[1]
            size_y = np_map.shape[0]
            resolution = map["resolution"]
            origin = map["origin"]

        self.np_map = np_map
        self.origin: Pose = origin
        self.resolution = resolution
        self.robot_base = Pose()
        self.size_x = size_x
        self.size_y = size_y
        self.ratio_x = ratio_x
        self.ratio_y = ratio_y

        self.set_robot_base(0, 0, 0)


if __name__ == "__main__":
    rclpy.init()
    node = test_navigator()
    map_param = node.get_map_ros2()
    map = ProcessedMap_test(map_param)
    print(map)
