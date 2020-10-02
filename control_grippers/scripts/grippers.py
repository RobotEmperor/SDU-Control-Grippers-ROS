#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

import math
from concurrent.futures.thread import ThreadPoolExecutor
import time
from typing import List, Callable

from i40.control.gripper_2f_controller import Gripper2FController
from i40.control.robot_controller import RobotController
from i40.control.robot_ft_controller import RobotFTController
from i40.kinematics.frame import Frame
from i40.kinematics.kinematics import Kinematics
from i40.kinematics.movable_frame import MovableFrame
from i40.loaders.cell_saver import CellSaver
from i40.loaders.hardware_loader import HardwareLoader
from i40.math.eaa import EAA
from i40.math.pose_6d import Pose6D
from i40.math.transform_3d import Transform3D
from i40.math.vector_3d import Vector3D
from i40.models.cell import Cell
from i40.models.device import Device
from i40.models.finger_pair import FingerPair
from i40.models.object import Object
from i40.persistence.persistencedriver import PersistenceDriver


from data_handling.wrs_persistence import WRSPersistence
from i40.loaders.cell_loader import CellLoader



class GrippersTask:
    def __init__(self, cell: Cell, mount_plate_name: str, persistence: PersistenceDriver):
        self._pool = ThreadPoolExecutor(6)

        # Load cell model
        self._persistence = persistence
        self._cell = cell
        print(cell.find_frame("cell"))

        # Find devices
        #self._robot_a_model = cell.find_serial_device("robot_a")
        #self._robot_b_model = cell.find_serial_device("robot_b")
        #self._robot_c_model = cell.find_serial_device("robot_c")
        self._gripper_a_model = cell.find_tree_device("gripper_a")
        self._gripper_b_model = cell.find_tree_device("gripper_b")

        def init_gripper(model):
            gripper = HardwareLoader.get_gripper(model, self._persistence)
            gripper.activate()
            return gripper

        gripper_a = self._pool.submit(init_gripper, self._gripper_a_model)
        gripper_b = self._pool.submit(init_gripper, self._gripper_b_model)
        print("Waiting for grippers..")
        while not gripper_a.done():
            pass
        while not gripper_b.done():
            pass

        self._gripper_a = gripper_a.result()
        self._gripper_b = gripper_b.result()

        print("Gripper Task initialized.")

    def move_grippper_a(self, value):
        self._gripper_a.move(value, 100, 100) #12.75

    def move_grippper_b(self, value):
        self._gripper_b.move(value, 100, 100)

def gripper_a_callback(data):
    global data_a
    if (data.data != data_a):
      rospy.loginfo("I heard A %f", data.data)
      grippers.move_grippper_a(data.data)
    data_a = data.data

def gripper_b_callback(data):
    global data_b
    if (data.data != data_b):
      rospy.loginfo("I heard B %f", data.data)
      grippers.move_grippper_b(data.data)
    data_b = data.data

if __name__=='__main__':
    persistence = WRSPersistence.driver()
    cell = CellLoader.load(persistence)
    grippers = GrippersTask(cell, "taskboard_plate", persistence)
    global data_a
    global data_b

    data_a = 0
    data_b = 0
    grippers.move_grippper_a(data_a)
    grippers.move_grippper_b(data_b)

    rospy.init_node('grippers', anonymous=True)
    rospy.Subscriber("/sdu/ur10e/gripper_a", Float64, gripper_a_callback)
    rospy.Subscriber("/sdu/ur10e/gripper_b", Float64, gripper_b_callback)

    rospy.spin()


