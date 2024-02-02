"""Driver class for Oculus controller.

"""

import time
from collections import namedtuple


import numpy as np
import rospy
from sensor_msgs.msg import Joy
import glfw
import sys
import copy 


from robosuite.devices import Device
from robosuite.utils.transform_utils import rotation_matrix
from scipy.spatial.transform import Rotation

sys.path.insert(0,'/home/rthom/Documents/Research/TRI/oculus_ws/src/oculus_reader/oculus_reader')

from reader import OculusReader

def mat2euler(mat):
    r = Rotation.from_matrix(mat)
    return r.as_euler('XYZ', degrees=False)

class OculusPolicy(): 
    """ Runs policy using Oculus controller commands"""
    def __init__(self):
        self.oculus_reader = OculusReader()
        self.demo_started = False 

    def get_oculus_state(self):
        poses, buttons = self.oculus_reader.get_transformations_and_buttons()
        while poses == {}: 
            poses, buttons = self.oculus_reader.get_transformations_and_buttons()
        return poses, buttons
        
    def initialize_policy(self, robot_pos_origin, robot_ori_origin):
        """Initializes the Oculus controller."""
        self.robot_pos_origin = robot_pos_origin
        self.robot_ori_origin = robot_ori_origin 
        poses, buttons = self.get_oculus_state()
        
        self.oculus_pos_origin, self.oculus_ori_origin = self.get_oculus_pose(poses)
        self.action_pos, self.action_ori, self.gripper_action = self.get_oculus_action(poses, buttons)
    
    def step(self, robot_pos_origin, robot_ori_origin): 
        poses, buttons = self.get_oculus_state()

        if buttons["RG"]: 
            if not self.demo_started: 
                self.demo_started = True 
                print("Demo started")
            self.action_pos, self.action_ori, self.gripper_action = self.get_oculus_action(poses, buttons)
        else:
            self.robot_pos_origin = robot_pos_origin 
            self.robot_ori_origin = robot_ori_origin
            self.oculus_pos_origin, self.oculus_ori_origin = self.get_oculus_pose(poses)
            self.action_pos = [0,0,0]
            self.action_ori = np.eye(3)
        
        return self.action_pos, self.action_ori, self.gripper_action

    def get_oculus_action(self, poses, buttons): 
        """Returns the action from the oculus controller."""
        oculus_pos = copy.deepcopy(poses['r'][:3,3])
        indices = [2,0,1]
        action_pos = oculus_pos[indices] - self.oculus_pos_origin[indices] + self.robot_pos_origin
        oculus_ori = copy.deepcopy(poses['r'][:3,:3])
        conv_mat = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        action_ori = (conv_mat @ oculus_ori) @ (conv_mat @ self.oculus_ori_origin).T @ self.robot_ori_origin
        gripper_action = buttons['rightTrig'][0]
        return action_pos, action_ori, gripper_action

    @staticmethod
    def get_oculus_pose(poses): 
        """Returns the oculus pose."""
        oculus_pos_origin = copy.deepcopy(poses['r'][:3,3])
        oculus_ori_origin = copy.deepcopy(poses['r'][:3,:3])
        return oculus_pos_origin, oculus_ori_origin
    
class Oculus(Device):

    def __init__(self, pos_sensitivity=1.0, rot_sensitivity=1.0, use_robotiq=False, drawer=False):

        # rospy.init_node("spacemouse_listener", anonymous=True)
        # rospy.Subscriber("spacenav/joy", Joy, self._get_spacemouse_commands)
        self.oculus_policy = OculusPolicy()
        self.oculus_policy.initialize_policy(np.array([0,0,0]), np.eye(3))


        # turn this value to true if using Robotiq gripper, false if using SSLIM
        self.use_robotiq = use_robotiq
        self.symmetric = True
        self.use_ori = True
        self.drawer = drawer

        self.alpha = 0.04
        self.alpha2 = 0.02

        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        # 6-DOF variables
        self.x, self.y, self.z = 0, 0, 0
        self.roll, self.pitch, self.yaw = 0, 0, 0

        if self.use_robotiq:
            self.dq = [0]
        else:
            self.dq = np.zeros(7)

        self.thumb_pos = 0.5
        self.pinch = False
        self.link_thumb = True

        self._display_controls()

        self.single_click_and_hold = False

        self._control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._reset_state = 0
        self.rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
        self._enabled = False
        self._buttons = [0.0, 0.0]

        self.last_button_state = [0, 0]

        self.last_clicked = [-1, -1]

    @staticmethod
    def _display_controls():
        """
        Method to pretty print controls.
        """

        def print_command(char, info):
            char += " " * (30 - len(char))
            print("{}\t{}".format(char, info))


    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """

        self._reset_state = 1
        self._enabled = False

        self.rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
        # Reset 6-DOF variables
        self.x, self.y, self.z = 0, 0, 0
        self.roll, self.pitch, self.yaw = 0, 0, 0
        # Reset control
        self._control = np.zeros(6)
        # Reset grasp
        self.single_click_and_hold = False
        if self.use_robotiq:
            self.dq = [-1]
        else:
            # self.sslim_state = 2
            self.dq = np.zeros(7)

    def _get_oculus_commands(self):

        action_pos, action_ori, gripper_action = self.oculus_policy.step(np.array([0,0,0]), np.eye(3))
        self.x, self.y, self.z = action_pos
        self.x *= self.pos_sensitivity
        self.y *= self.pos_sensitivity
        self.z *= self.pos_sensitivity
        # rpy = mat2euler(action_ori)
        # self.roll, self.pitch, self.yaw = rpy
        self.rotation = action_ori

        self._control = [
            self.x,
            self.y,
            self.z,
            self.roll,
            self.pitch,
            self.yaw,
        ]

        print(self._control)

    def on_press(self, window, key, scancode, action, mods):
        """
        Key handler for key presses.

        Args:
            window: [NOT USED]
            key (int): keycode corresponding to the key that was pressed
            scancode: [NOT USED]
            action: [NOT USED]
            mods: [NOT USED]
        """
        # controls for moving thumb
        if key == glfw.KEY_Q:
            self.thumb_pos = 1
        elif key == glfw.KEY_W:
            self.thumb_pos = -1
        elif key == glfw.KEY_A:
            self.pinch = True

            self.dq[1] = 0
            self.dq[3] = 0
            self.dq[6] = 0

        elif key == glfw.KEY_S:
            self.pinch = False

            self.dq[1] = self.dq[0] * (self.alpha2 / self.alpha)
            self.dq[3] = self.dq[2] * (self.alpha2 / self.alpha)
            if self.link_thumb:
                self.dq[6] = self.dq[5] * (self.alpha2 / self.alpha)

        elif key == glfw.KEY_Z:
            self.link_thumb = False
            self.dq[5] = 0
            self.dq[6] = 0

        elif key == glfw.KEY_X:
            self.link_thumb = True
            self.dq[5] = self.dq[2]
            if self.pinch:
                self.dq[6] = 0
            else:
                self.dq[6] = self.dq[5] * (self.alpha2 / self.alpha)

        elif key == glfw.KEY_B:
            self._reset_internal_state()

        elif key == glfw.KEY_DELETE:
            raise ValueError("Exit training!")

    def start_control(self):
        """
        Method that should be called externally before controller can
        start receiving commands.
        """
        self._reset_internal_state()
        self._reset_state = 0
        self._enabled = True

    def get_controller_state(self):
        """
        Grabs the current state of the 3D mouse.


        Returns:
            dict: A dictionary containing dpos, orn, unmodified orn, grasp, and reset
        """
        self._get_oculus_commands()
        dpos = self.control[:3] * 0.008 * self.pos_sensitivity
        roll, pitch, yaw = self.control[3:] * 0.008 * self.rot_sensitivity

        # convert RPY to an absolute orientation
        drot1 = rotation_matrix(angle=-pitch, direction=[1.0, 0, 0], point=None)[:3, :3]
        drot2 = rotation_matrix(angle=roll, direction=[0, 1.0, 0], point=None)[:3, :3]
        drot3 = rotation_matrix(angle=yaw, direction=[0, 0, 1.0], point=None)[:3, :3]

        self.rotation = self.rotation.dot(drot1.dot(drot2.dot(drot3)))

        if self.use_robotiq:
            if self._buttons[0]:
                self.dq[0] = min(1, self.dq[0] + 0.1)
            if self._buttons[1]:
                self.dq[0] = max(-1, self.dq[0] - 0.1)
                
            if self._buttons[0]:
                if self.dq[0] <= 1.5:
                    self.dq[0] += self.alpha * self.pos_sensitivity
                    self.dq[2] += self.alpha * self.pos_sensitivity
                    if self.link_thumb:
                        self.dq[5] += self.alpha * self.pos_sensitivity

                    if not self.pinch:
                        self.dq[1] += self.alpha2 * self.pos_sensitivity
                        self.dq[3] += self.alpha2 * self.pos_sensitivity
                        if self.link_thumb:
                            self.dq[6] += self.alpha2 * self.pos_sensitivity

            if self._buttons[1]:
                if self.dq[0] >= -1.5:
                    self.dq[0] -= self.alpha * self.pos_sensitivity
                    self.dq[2] -= self.alpha * self.pos_sensitivity
                    if self.link_thumb:
                        self.dq[5] -= self.alpha * self.pos_sensitivity

                    if not self.pinch:
                        self.dq[1] -= self.alpha2 * self.pos_sensitivity
                        self.dq[3] -= self.alpha2 * self.pos_sensitivity
                        if self.link_thumb:
                            self.dq[6] -= self.alpha2 * self.pos_sensitivity

            if not self.symmetric:
                self.dq = np.maximum(self.dq, np.zeros_like(self.dq))
            self.dq[4] = self.thumb_pos

        return dict(
            dpos=dpos,
            rotation=self.rotation,
            raw_drotation=np.array([roll, pitch, yaw]),
            grasp=self.control_gripper,
            dq=self.dq,
            reset=self._reset_state,
        )

    @property
    def control(self):
        """
        Grabs current pose of Spacemouse


        Returns:
            np.array: 6-DoF control value
        """
        return np.array(self._control)

    @property
    def control_gripper(self):
        """
        Maps internal states into gripper commands.


        Returns:
            float: Whether we're using single click and hold or not
        """
        if self.single_click_and_hold:
            return 1.0
        return 0


if __name__ == "__main__":
    oculus = Oculus()
    for i in range(100):
        print(space_mouse.control, space_mouse.control_gripper)
        time.sleep(0.02)
