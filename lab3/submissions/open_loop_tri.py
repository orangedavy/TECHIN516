import os
import time

# Move forward for 1.5m using open-loop strategy.
timenow = time.time()
os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.17453]'")
print("Speed: 0.0, Start timer: 0.0")
# the first pub latches for 3 seconds before continuing with next line
while time.time() - timenow < 3.0: 
    pass
print("Speed 0.0 Stop timer: ", time.time() - timenow)
os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")

os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")
print("Speed: 0.5, Start timer: 0.0")
# the first pub latches for 3 seconds before continuing with next line
while time.time() - timenow < 3.0: 
    pass
print("Speed 0.0 Stop timer: ", time.time() - timenow)
os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")

os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.6981]'")
print("Speed: 0.0, Start timer: 0.0")
# the first pub latches for 3 seconds before continuing with next line
while time.time() - timenow < 3.0: 
    pass
print("Speed 0.0 Stop timer: ", time.time() - timenow)
os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")

os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")
print("Speed: 0.5, Start timer: 0.0")
# the first pub latches for 3 seconds before continuing with next line
while time.time() - timenow < 3.0: 
    pass
print("Speed 0.0 Stop timer: ", time.time() - timenow)
os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")

os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.3491]'")
print("Speed: 0.0, Start timer: 0.0")
# the first pub latches for 3 seconds before continuing with next line
while time.time() - timenow < 3.0: 
    pass
print("Speed 0.0 Stop timer: ", time.time() - timenow)
os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")

os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")
print("Speed: 0.5, Start timer: 0.0")
# the first pub latches for 3 seconds before continuing with next line
while time.time() - timenow < 3.0: 
    pass
print("Speed 0.0 Stop timer: ", time.time() - timenow)
os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")
