#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

GRID_ROWS = 100
GRID_COLS = 100

METERS_LENGTH = 5.0
MAX_X = METERS_LENGTH / 2.0
MIN_X = -METERS_LENGTH / 2.0
MAX_Y = METERS_LENGTH / 2.0
MIN_Y = -METERS_LENGTH / 2.0

GRID_RESOLUTION = METERS_LENGTH / GRID_ROWS

BELIEF_INC = 1
BELIEF_DEC = 1
BELIEF_THRESH = 50

MAX_BELIEF = 100
MIN_BELIEF = 0

pub = None
grid = None

def initOccupancyGridMetaData():
    grid = OccupancyGrid()
    
    grid.info = MapMetaData(
        width=GRID_COLS,
        height=GRID_ROWS,
        resolution=GRID_RESOLUTION,
        origin=Pose(
            position=Point(
                x=MIN_X,
                y=MAX_Y
            )
        )
    )

    grid.data = [0 for i in range(0, GRID_COLS * GRID_ROWS)]
    perm_grid = [0 for i in range(0, GRID_COLS * GRID_ROWS)]

def processPointCloud(pc):
    grid.header = pc.header

    hit_grid = [[False for c in range(0, GRID_COLS)] for r in range(0, GRID_ROWS)]

    # Increment all cells that the laser data hits
    for point in pc.points:
        if point.x > MAX_X or point.x < MIN_X or point.y > MAX_Y or point.y < MIN_Y:
            continue

        col = point.x + MIN_X / float(GRID_COLS)
        row = point.y + MIN_Y / float(GRID_ROWS)

        perm_data[row * GRID_COLS + col] += BELIEF_INC
        hit_grid[row][col] = True

    # Decrement all cells that don't have hits
    for r in range(0, GRID_ROWS):
        for c in range(0, GRID_COLS):
            if hit_grid[r][c]:
                perm_data[r * GRID_COLS + c] -= BELIEF_DEC

    # Bound the values in the cells   
    for i in range(len(perm_data)):
        perm_data[i] = MIN_BELIEF if perm_grid[i] < MIN_BELIEF else perm_grid[i]
        perm_data[i] = MAX_BELIEF if perm_grid[i] > MAX_BELIEF else perm_grid[i]
 
    # Threshold the values of the cells to decide if it's free / unknown / obstacle
    for i in range(len(grid.data)):
        grid.data[i] = 0 if perm_gri[i] < BELIEF_THRESH else 1

    return grid

def callback(data):
    pub.publish(processPointCloud(data))

def init():
    initOccupancyGridMetaData()
    pub = rospy.Publisher('/occupancy_grid', OccupancyGrid)
    rospy.init_node('occupancy_grid_maker')
    rospy.Subscriber('/evn_data/camera/pointcloud', PointCloud, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        init();
    except rospy.ROSInterruptException:
        pass

