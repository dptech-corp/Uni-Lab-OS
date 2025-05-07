import rclpy

# Initialize ROS communications for a given context
if(rclpy.ok() == False):
    rclpy.init()
else:
    print("rclpy already initiated")

from unilabos.devices.opsky_Raman.opsky_ATR30007 import ATR30007
from unilabos_msgs.action import SendCmd
from std_msgs.msg import Float64, String, Int16
from unilabos.ros.device_node_wrapper import ros2_device_node
import clr
import os
# dll_path = r'D:\UniLab\code\DHElecChem\release64'
# eccore_dll_path = os.path.join(dll_path, 'ECCore.dll')
# os.environ["PATH"] = dll_path + os.pathsep + os.environ["PATH"]
# clr.AddReference(eccore_dll_path)
# from ECCore import ElecMachines

ROS2_ATR30007 = ros2_device_node(
    ATR30007, 
    #status_types={'machine_id': Int16, 'status': String}, 
    action_value_mappings={'opsky_cmd': {
        'type': SendCmd, 
        'goal': {'command': 'command'},
        'feedback': {},
        'result': {'success': 'success'}}
    }
)

device = ROS2_ATR30007(device_id='ATR30007_1')
rclpy.spin(device.ros_node_instance)