"""Driver class for SpaceMouse controller.


This class provides a driver support to SpaceMouse on macOS.
In particular, we assume you are using a SpaceMouse Wireless by default.


To set up a new SpaceMouse controller:
   1. Download and install driver from https://www.3dconnexion.com/service/drivers.html
   2. Install hidapi library through pip
      (make sure you run uninstall hid first if it is installed).
   3. Make sure SpaceMouse is connected before running the script
   4. (Optional) Based on the model of SpaceMouse, you might need to change the
      vendor id and product id that correspond to the device.


For Linux support, you can find open-source Linux drivers and SDKs online.
   See http://spacenav.sourceforge.net/


"""


import threading
import time
from collections import namedtuple


import numpy as np
import rospy
from sensor_msgs.msg import Joy


try:
   import hid
except ModuleNotFoundError as exc:
   raise ImportError(
       "Unable to load module hid, required to interface with SpaceMouse. "
       "Only macOS is officially supported. Install the additional "
       "requirements with `pip install -r requirements-extra.txt`"
   ) from exc


from robosuite.devices import Device
from robosuite.utils.transform_utils import rotation_matrix


AxisSpec = namedtuple("AxisSpec", ["channel", "byte1", "byte2", "scale"])


SPACE_MOUSE_SPEC = {
   "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
   "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
   "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
   "roll": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
   "pitch": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
   "yaw": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
}




def to_int16(y1, y2):
   """
   Convert two 8 bit bytes to a signed 16 bit integer.


   Args:
       y1 (int): 8-bit byte
       y2 (int): 8-bit byte


   Returns:
       int: 16-bit integer
   """
   x = (y1) | (y2 << 8)
   if x >= 32768:
       x = -(65536 - x)
   return x




def scale_to_control(x, axis_scale=350.0, min_v=-1.0, max_v=1.0):
   """
   Normalize raw HID readings to target range.


   Args:
       x (int): Raw reading from HID
       axis_scale (float): (Inverted) scaling factor for mapping raw input value
       min_v (float): Minimum limit after scaling
       max_v (float): Maximum limit after scaling


   Returns:
       float: Clipped, scaled input from HID
   """
   x = x / axis_scale
   x = min(max(x, min_v), max_v)
   return x




def convert(b1, b2):
   """
   Converts SpaceMouse message to commands.


   Args:
       b1 (int): 8-bit byte
       b2 (int): 8-bit byte


   Returns:
       float: Scaled value from Spacemouse message
   """
   return scale_to_control(to_int16(b1, b2))




class SpaceMouse(Device):
   """
   A minimalistic driver class for SpaceMouse with HID library.


   Note: Use hid.enumerate() to view all USB human interface devices (HID).
   Make sure SpaceMouse is detected before running the script.
   You can look up its vendor/product id from this method.


   Args:
       vendor_id (int): HID device vendor id
       product_id (int): HID device product id
       pos_sensitivity (float): Magnitude of input position command scaling
       rot_sensitivity (float): Magnitude of scale input rotation commands scaling
   """


   def __init__(self, vendor_id=9583, product_id=50735, pos_sensitivity=1.0, rot_sensitivity=1.0):


       print("Opening SpaceMouse device")
       rospy.init_node('spacemouse_listener', anonymous=True)
       rospy.Subscriber("spacenav/joy", Joy, self._get_spacemouse_commands)


       self.pos_sensitivity = pos_sensitivity
       self.rot_sensitivity = rot_sensitivity


       # 6-DOF variables
       self.x, self.y, self.z = 0, 0, 0
       self.roll, self.pitch, self.yaw = 0, 0, 0

       self.dq = np.zeros(7)


       self._display_controls()


       self.single_click_and_hold = False


       self._control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
       self._reset_state = 0
       self.rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
       self._enabled = False
       self._buttons = [0.0, 0.0]




   @staticmethod
   def _display_controls():
       """
       Method to pretty print controls.
       """


       def print_command(char, info):
           char += " " * (30 - len(char))
           print("{}\t{}".format(char, info))


       print("")
       print_command("Control", "Command")
       print_command("Right button", "reset simulation")
       print_command("Left button (hold)", "close gripper")
       print_command("Move mouse laterally", "move arm horizontally in x-y plane")
       print_command("Move mouse vertically", "move arm vertically")
       print_command("Twist mouse about an axis", "rotate arm about a corresponding axis")
       print_command("ESC", "quit")
       print("")


   def _reset_internal_state(self):
       """
       Resets internal state of controller, except for the reset signal.
       """
       self.rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
       # Reset 6-DOF variables
       self.x, self.y, self.z = 0, 0, 0
       self.roll, self.pitch, self.yaw = 0, 0, 0
       # Reset control
       self._control = np.zeros(6)
       # Reset grasp
       self.single_click_and_hold = False


   def _get_spacemouse_commands(self, data):
       self._axes = data.axes
       self._buttons = data.buttons

       self.y = self._axes[0]
       self.x = self._axes[1] * -1
       self.z = self._axes[2] 
       self.roll = self._axes[3]
       self.pitch = self._axes[4] * -1
       self.yaw = self._axes[5]
       self._control = [
            self.x,
            self.y,
            self.z,
            self.roll,
            self.pitch,
            self.yaw,
        ]


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
       dpos = self.control[:3] * 0.005 * self.pos_sensitivity
       roll, pitch, yaw = self.control[3:] * 0.005 * self.rot_sensitivity


       # convert RPY to an absolute orientation
       drot1 = rotation_matrix(angle=-pitch, direction=[1.0, 0, 0], point=None)[:3, :3]
       drot2 = rotation_matrix(angle=roll, direction=[0, 1.0, 0], point=None)[:3, :3]
       drot3 = rotation_matrix(angle=yaw, direction=[0, 0, 1.0], point=None)[:3, :3]


       self.rotation = self.rotation.dot(drot1.dot(drot2.dot(drot3)))
       
       alpha = 0.05
       alpha2 = 0.03
       if self._buttons[0]:
           
            if self.dq[0] <= 1.5:
                self.dq[0] += alpha * self.pos_sensitivity
                self.dq[2] += alpha * self.pos_sensitivity
                self.dq[5] += alpha * self.pos_sensitivity

            # if self.dq[1] <= 1:
                self.dq[1] += alpha2 * self.pos_sensitivity
                self.dq[3] += alpha2 * self.pos_sensitivity
                self.dq[6] += alpha2 * self.pos_sensitivity
                # self.dq[0:5] += alpha * self.pos_sensitivity
                # self.dq[5:]  += alpha * self.pos_sensitivity

       if self._buttons[1]:
           if self.dq[0] >= -1.5:
                self.dq[0] -= alpha * self.pos_sensitivity
                self.dq[2] -= alpha * self.pos_sensitivity
                self.dq[5] -= alpha * self.pos_sensitivity
        #    if self.dq[1] >= -1:
                self.dq[1] -= alpha2 * self.pos_sensitivity
                self.dq[3] -= alpha2 * self.pos_sensitivity
                self.dq[6] -= alpha2 * self.pos_sensitivity
            # self.dq[0:5] -=  alpha * self.pos_sensitivity
            # self.dq[5:]  -=  alpha * self.pos_sensitivity

        
       self.dq[4] = 1
       
       return dict(
           dpos=dpos,
           rotation=self.rotation,
           raw_drotation=np.array([roll, pitch, yaw]),
           grasp=self.control_gripper,
           dq = self.dq,
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
   space_mouse = SpaceMouse()
   for i in range(100):
       print(space_mouse.control, space_mouse.control_gripper)
       time.sleep(0.02)