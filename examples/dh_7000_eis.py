import rclpy

# Initialize ROS communications for a given context
if(rclpy.ok() == False):
    rclpy.init()
else:
    print("rclpy already initiated")

from unilabos.devices.dh_electrochem.dh_7000 import DH7000
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

ROS2_DH7000 = ros2_device_node(
    DH7000, 
    status_types={'machine_id': Int16, 'status': String}, 
    action_value_mappings={'dh_cmd': {
        'type': SendCmd, 
        'goal': {'command': 'command'},
        'feedback': {},
        'result': {'success': 'success'}}
    }
)

device = ROS2_DH7000(device_id='DH7000_1')
rclpy.spin(device.ros_node_instance)