import herbpy
import openravepy
import numpy as np
import os

from catkin.find_in_workspaces import find_in_workspaces
objects_path = find_in_workspaces(
    search_dirs=['share'],
    project='pr_ordata',
    path='data/objects',
    first_match_only=True)[0]

env, robot = herbpy.initialize(sim=True, attach_viewer='rviz')
table_file = os.path.join(objects_path, 'table.kinbody.xml')
table = env.ReadKinBodyXMLFile(table_file)
env.AddKinBody(table)

herb_pose = robot.GetTransform()

table_pose = np.array([[1., 0.,  0., 2],
                    [0., 0., -1., 2],
                    [0., 1.,  0., 0.0], 
                    [0., 0.,  0., 1.]])
table_pose[:3,3] = herb_pose[:3,3]
table_pose[1,3] -= 0.75
table.SetTransform(table_pose)

#raw_input('press enter to begin planning')
#robot.PlanToNamedConfiguration('home',execute=True)


#Initialize octomap
openravepy.RaveLoadPlugin("or_octomap")
sensor_server = openravepy.RaveCreateSensorSystem(env,"or_octomap")
sensor_server.SendCommand("Enable")
sensor_server.SendCommand("Mask " + robot.GetName())
#Create collision checker
'''
collision_checker = openravepy.RaveCreateCollisionChecker(env,"or_octomap_checker")
collision_checker_old = env.GetCollisionChecker()
env.SetCollisionChecker(collision_checker)
'''


#raw_input('press enter to begin end planning')
#plan to some pose
#end_config = np.array([2.336375162301677, -0.7521315673852575, 2.6228803736881265, 0.5508421753993923, -0.01587398287783462, -0.2861305355919238, -0.7392284125047297])
#robot.right_arm.PlanToConfiguration(end_config,execute=True)
