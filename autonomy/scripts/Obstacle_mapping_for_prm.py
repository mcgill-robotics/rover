import rospy
from geometry_msgs.msg import Point32
import time
import json

# This is a Subscriber that retrieves obstacle and location information, which then saves it as a json file
map_grid = {}
def sub(point: Point32):
    x = round(point.x, 1)
    y = round(point.y, 1)
    if f"{x} {y}" not in map_grid:
        map_grid[f"{x} {y}"] = True
        with open ('autonomy/scripts/prm/obstacles.json', "w") as jsonfile:
            json.dump({"map":map_grid, "location": None}, jsonfile)

while True:
    rospy.init_node('prm_obstacle_sub')
    sub = rospy.Subscriber('prm_obstacles', Point32, sub)
    time.sleep(5)
    rospy.spin()

