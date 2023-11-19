from rclpy.node import Node
from asl_tb3_msgs.msg import TurtleBotState
from asl_tb3_lib.grids import StochOccupancyGrid2D
from nav_msgs.msg import OccupancyGrid, Path
from scipy.signal import convolve2d
import numpy as np
import typing as T
from std_msgs.msg import Bool

class Explorer(Node):
    def __init__(self) -> None:
        # give it a default node name
        super().__init__("explorer_node")
        self.get_logger().info('Explorer node has been created')
        self.occupancy: T.Optional[StochOccupancyGrid2D] = None
        self.state: T.Optional[TurtleBotState] = None
        
        self.goal_pub = self.create_publisher(TurtleBotState, "/goal_pose", 1)
        self.nav_sub = self.create_subscription(Bool,"/nav_success",self.explore_callback,10)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.state_sub = self.create_subscription(TurtleBotState, "/state", self.state_callback, 10)


        #do the first navigation
        self.frontier_explore()

    def frontier_explore(self):
        window_size = 13    # defines the window side-length for neighborhood of cells to consider for heuristics
        occupied_mask = (self.occupancy.probs >= 0.5)
        unoccupied_mask = ((self.occupancy.probs < 0.5) & (self.occupancy.probs != -1))
        unknown_mask = (self.occupancy.probs == -1)
        
        # do the convolution
        occupied_count = convolve2d(occupied_mask, np.ones((window_size, window_size)), mode='same',boundary = 'fill',fillvalue = 0)
        unoccupied_count = convolve2d(unoccupied_mask, np.ones((window_size, window_size)), mode='same',boundary = 'fill',fillvalue = 0)
        unknown_count = convolve2d(unknown_mask, np.ones((window_size, window_size)), mode='same',boundary = 'fill',fillvalue = 0)

        # find the potential states
        frontier_list = []
        min_distance = 99999999999
        closest_state = np.array([0., 0.])
        for i in range(unknown_count.shape[0]):
            for j in range(unknown_count.shape[1]):
                if(occupied_count[i,j] == 0 and unoccupied_count[i,j] >= 2 and unknown_count[i,j] >= 3):
                    state_frontier = self.occupancy.grid2state(np.array([j,i]))
                    distance = np.linalg.norm(state_frontier - self.state)
                    if(distance < min_distance):
                        min_distance = distance
                        closest_state = state_frontier

                    frontier_list.append(state_frontier)
        frontier_states = np.array(frontier_list)
        print("closest_state", closest_state)
        print("min_distance", min_distance)
        goal_state = TurtleBotState(
            x = closest_state[0],
            y = closest_state[1]
        )
        #should ultimately publish a goal pose
        self.goal_pub.publish(goal_state)


    def explore_callback(self,msg:Bool):
        if(msg.data):
            # do the exploration
            self.frontier_explore()

    def map_callback(self, msg: OccupancyGrid) -> None:
        """ Callback triggered when the map is updated

        Args:
            msg (OccupancyGrid): updated map message
        """
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=9,
            probs=msg.data,
        )

    def state_callback(self, msg: TurtleBotState) -> None:
        """ callback triggered when receiving latest turtlebot state

        Args:
            msg (TurtleBotState): latest turtlebot state
        """
        self.state = msg

