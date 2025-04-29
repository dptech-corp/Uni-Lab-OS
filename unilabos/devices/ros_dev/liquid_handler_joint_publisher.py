import rclpy
import json
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from ilabos_msgs.action import SendCmd
from rclpy.action.server import ServerGoalHandle
from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster, Buffer, TransformListener 


class LiquidHandlerJointPublisher(BaseROS2DeviceNode):
    def __init__(self,device_id:str, joint_config:dict,resource_tracker):
        super().__init__(
            driver_instance=self,
            device_id=device_id,
            status_types={},
            action_value_mappings={},
            hardware_interface={},
            print_publish=False,
            resource_tracker=resource_tracker,  
        )  
        # self.station_name   = station_name
        # self.dev_name       = dev_name
        self.j_msg          = JointState()
        # self.j_msg.name     = joint_names
        self.j_msg.name     = [device_id+x for x in joint_config.keys()]
        self.rate           = 50
        self.j_msg.position = [0.0 for i in range(len(joint_config.keys()))]
        self.tf_buffer      = Buffer()
        self.tf_listener    = TransformListener(self.tf_buffer, self)
        self.j_pub          = self.create_publisher(JointState,'/joint_states',10)
        self.create_timer(0.02,self.sim_joint_pub_callback)
        self.j_action       = ActionServer(
            self,
            SendCmd,
            "joint",
            self.sim_joint_action_callback,
            result_timeout=5000
        )

    def sim_joint_action_callback(self,goal_handle: ServerGoalHandle):
        """Move a single joint

        Args:
            command: A JSON-formatted string that includes joint_name, speed, position

                    joint_name (str): The name of the joint to move
                    speed (float): The speed of the movement, speed > 0
                    position (float): The position to move to

        Returns:
            None
        """
        result = SendCmd.Result()
        cmd_str = str(goal_handle.request.command).replace('\'','\"')
        # goal_handle.execute()

        try:
            cmd_dict = json.loads(cmd_str)
            self.move_joint(**cmd_dict)
            result.success = True
            goal_handle.succeed()
            
        except Exception as e:
            print(e)
            goal_handle.abort()
            result.success = False

        return result


    def move_joint(self, joint_name, speed, position):
        
        # joint_index = self.j_msg.name.index(self.station_name+self.dev_name+joint_name)
        joint_index = self.j_msg.name.index(joint_name)
        distance = position - self.j_msg.position[joint_index]
        if distance == 0:
            return
        flag = abs(distance)/distance
        
        while abs(distance)>speed/self.rate:
            
            self.j_msg.position[joint_index] += flag*speed/self.rate
            self.sim_joint_pub_callback()
            time.sleep(0.02)
            distance = position - self.j_msg.position[joint_index]

        self.j_msg.position[joint_index] = position
        self.sim_joint_pub_callback()


    def sim_joint_pub_callback(self):
        self.j_msg.header.stamp = self.get_clock().now().to_msg()
        self.j_pub.publish(self.j_msg)

def main():

    joint_json:dict = json.load(open("device_data.json", encoding='utf-8'))
    joint_action_list = {}
    rclpy.init()
    executor = MultiThreadedExecutor()
    for station_name,dev_dict in joint_json.items():
        for dev_name,joint_names in dev_dict.items():
            joint_action_list[station_name+'_'+dev_name] = SimJointPublisher(station_name,dev_name,joint_names)
            executor.add_node(joint_action_list[station_name+'_'+dev_name])

    executor.spin()

if __name__ == '__main__':
    main()