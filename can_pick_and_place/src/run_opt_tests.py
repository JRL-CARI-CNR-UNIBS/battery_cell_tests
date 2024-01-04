#!/usr/bin/env python3

import rospy
import rospkg
import sys
from skills_util_msgs.srv import RunTree
from pybullet_utils.srv import SpawnModel
from pybullet_utils.srv import DeleteModel
from pybullet_utils.srv import SaveState
from geometry_msgs.msg import Pose
import pyexcel_ods3 as od

rospack = rospkg.RosPack()
path = rospack.get_path('battery_cell_tests')
path = path + '/can_pick_and_place/src/python_classes'
sys.path.append(path)

import personal_class
import real_personal_class


if __name__ == '__main__':

    tests_type = 'simulation'

    rospy.init_node('optimizer_tests', anonymous=True)
    rospy.set_param('/optimization_end', False)

    spawn_model_clnt = rospy.ServiceProxy('/pybullet_spawn_model', SpawnModel)
    delete_model_clnt = rospy.ServiceProxy('/pybullet_delete_model', DeleteModel)

    start_rl_params = rospy.get_param('RL_params')

    run_tree_clnt = rospy.ServiceProxy('/skills_util/run_tree', RunTree)

    task_name = rospy.get_param('/task_name')

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('battery_cell_tests')
    tree_folder_path = package_path + '/' + task_name + '/config/trees'

    tree_name = rospy.get_param('/tree_name')

    run_tree_clnt.call('init_tree', [tree_folder_path])

    object_name = []
    object_name.append('can')
    model_name = []
    model_name.append('can')
    pose = []
    model_pose = Pose()
    model_pose.position.x = 0.0
    model_pose.position.y = -1.3
    model_pose.position.z = 1.0
    model_pose.orientation.x = 0.0
    model_pose.orientation.y = 0.0
    model_pose.orientation.z = 1.0
    model_pose.orientation.w = 0.0
    pose.append(model_pose)

    spawn_model_clnt.call(object_name, model_name, pose, False)
    print("Model spawned")

    save_state_clnt = rospy.ServiceProxy('/pybullet_save_state', SaveState)
    save_state_clnt.call('start')

    test_names = ['test_1']

    iteration_number = 100

    for test_name in test_names:
        rospy.set_param('RL_params', start_rl_params)

        print('Test: ' + test_name)

        if (tests_type == 'simulation'):
            pers_c = personal_class.personal_optimizer(package_path, task_name, tree_name, test_name, iteration_number)
            if not pers_c.run_optimization():
                rospy.logerr('Failure during optimization process')
                exit(1)
        elif (tests_type == 'real'):
            pers_c = real_personal_class.real_personal_optimizer(package_path, task_name, tree_name, test_name, iteration_number)
            if not pers_c.run_optimization():
                rospy.logerr('Failure during optimization process')
                exit(1)

        ros_data = rospy.get_param(test_name)
        data = []
        data.append(ros_data[0].keys())
        for index in range(len(ros_data)):
            data.append(ros_data[index].values())
        if (tests_type == 'simulation'):
            data_ods = od.get_data(package_path + '/' + task_name + "/data/simulation_tests.ods")
            data_ods.update({test_name: data})
            od.save_data(package_path + '/' + task_name + "/data/simulation_tests.ods", data_ods)
        if (tests_type == 'real'):
            data_ods = od.get_data(package_path + '/' + task_name + "/data/real_tests.ods")
            data_ods.update({test_name: data})
            od.save_data(package_path + '/' + task_name + "/data/real_tests.ods", data_ods)

    rospy.set_param('/optimization_end', True)

    delete_model_clnt.call(object_name)
