import numpy as np
import pandas as pd
from pick_and_place import PickAndPlace
import copy
from pyquaternion import Quaternion

repo = pd.read_csv('data.csv',sep=',',header=0)
data = np.array((repo['x'].values, repo['y'].values, repo['z'].values))
print("Number of data points: ", data.shape[1])
qStart = Quaternion(array=np.array([0.499, 0.500, 0.500, 0.501]))
qEnd = Quaternion(array=np.array([1.000, 0.000, -0.001, 0.000, ]))
qList = []
# Quickest (not most effective) way to get the items from the intermediates method since 
# its output is a generator of objects and has no easy way to get elements from it
for q in Quaternion.intermediates(qStart, qEnd, data.shape[1]-2, include_endpoints=True):
    qList.append(q.elements)
example = PickAndPlace()
waypoints = []
current_pose = example.get_cartesian_pose()

for points in range(data.shape[1]):
    current_pose.position.x = data[0,points]
    current_pose.position.y = data[1,points]
    current_pose.position.z = data[2,points]
    current_pose.orientation.w = qList[points][0]
    current_pose.orientation.x = qList[points][1]
    current_pose.orientation.y = qList[points][2]
    current_pose.orientation.z = qList[points][3]
    waypoints.append(copy.deepcopy(current_pose))
    
print(len(waypoints), waypoints)
cartesian_plan, fraction = example.plan_cartesian_path(waypoints)
example.execute_plan(cartesian_plan)


