import copy
import rclpy
import json
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from unilabos_msgs.action import SendCmd
from rclpy.action.server import ServerGoalHandle
from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster, Buffer, TransformListener 


class LiquidHandlerJointPublisher(BaseROS2DeviceNode):
    def __init__(self,device_id:str, joint_config:dict, lh_id:str,resource_tracker, rate=50):
        super().__init__(
            driver_instance=self,
            device_id=device_id,
            status_types={},
            action_value_mappings={},
            hardware_interface={},
            print_publish=False,
            resource_tracker=resource_tracker,  
        )  

        # joint_config_dict = {
        #     "joint_names":[
        #         "first_joint",
        #         "second_joint",
        #         "third_joint",
        #         "fourth_joint"
        #     ],
        #     "y":{
        #         "first_joint":{
        #         "factor":-1,
        #         "offset":0.0
        #         }
        #     },
        #     "x":{
        #         "second_joint":{
        #         "factor":-1,
        #         "offset":0.0
        #         }
        #     },
        #     "z":{
        #         "third_joint":{
        #         "factor":1,
        #         "offset":0.0
        #         },
        #         "fourth_joint":{
        #         "factor":1,
        #         "offset":0.0
        #         }
        #     }
        # }

        self.j_msg          = JointState()
        self.lh_id          = lh_id
        # self.j_msg.name     = joint_names
        self.joint_config   = joint_config
        self.j_msg.position = [0.0 for i in range(len(joint_config['joint_names']))]
        self.j_msg.name     = [f"{self.lh_id}_{x}" for x in joint_config['joint_names']]
        # self.joint_config   = joint_config_dict
        # self.j_msg.position = [0.0 for i in range(len(joint_config_dict['joint_names']))]
        # self.j_msg.name     = [f"{self.lh_id}_{x}" for x in joint_config_dict['joint_names']]
        self.rate           = rate
        self.tf_buffer      = Buffer()
        self.tf_listener    = TransformListener(self.tf_buffer, self)
        self.j_pub          = self.create_publisher(JointState,'/joint_states',10)
        self.create_timer(0.02,self.lh_joint_pub_callback)
        self.j_action       = ActionServer(
            self,
            SendCmd,
            "joint",
            self.lh_joint_action_callback,
            result_timeout=5000
        )

    def lh_joint_action_callback(self,goal_handle: ServerGoalHandle):
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
            self.move_joints(**cmd_dict)
            result.success = True
            goal_handle.succeed()
            
        except Exception as e:
            print(e)
            goal_handle.abort()
            result.success = False

        return result
    def inverse_kinematics(self, x, y, z, 
                           x_joint:dict, 
                           y_joint:dict, 
                           z_joint:dict   ):
        """
        将x、y、z坐标转换为对应关节的位置
        
        Args:
            x (float): x坐标
            y (float): y坐标
            z (float): z坐标
            x_joint (dict): x轴关节配置，包含plus和offset
            y_joint (dict): y轴关节配置，包含plus和offset
            z_joint (dict): z轴关节配置，包含plus和offset
            
        Returns:
            dict: 关节名称和对应位置的字典
        """
        joint_positions = copy.deepcopy(self.j_msg.position)
        
        # 处理x轴关节
        for joint_name, config in x_joint.items():
            index = self.j_msg.name.index(f"{self.lh_id}_{joint_name}")
            joint_positions[index] = x * config["factor"] + config["offset"]
            
        # 处理y轴关节
        for joint_name, config in y_joint.items():
            index = self.j_msg.name.index(f"{self.lh_id}_{joint_name}")
            joint_positions[index] = y * config["factor"] + config["offset"]
            
        # 处理z轴关节
        for joint_name, config in z_joint.items():
            index = self.j_msg.name.index(f"{self.lh_id}_{joint_name}")
            joint_positions[index] = z * config["factor"] + config["offset"]
            
        
        return joint_positions


    def move_joints(self, resource_name, link_name, speed, x_joint=None, y_joint=None, z_joint=None):
        
        transform = self.tf_buffer.lookup_transform(
                    link_name,
                    resource_name,
                    rclpy.time.Time()
                )
        x,y,z = transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z
        if x_joint is None:
            x_joint_config = next(iter(self.joint_config['x'].items()))
        elif x_joint in self.joint_config['x']:
            x_joint_config = self.joint_config['x'][x_joint]
        else:
            raise ValueError(f"x_joint {x_joint} not in joint_config['x']")
        if y_joint is None:
            y_joint_config = next(iter(self.joint_config['y'].items()))
        elif y_joint in self.joint_config['y']:
            y_joint_config = self.joint_config['y'][y_joint]
        else:
            raise ValueError(f"y_joint {y_joint} not in joint_config['y']")
        if z_joint is None:
            z_joint_config = next(iter(self.joint_config['z'].items()))
        elif z_joint in self.joint_config['z']:
            z_joint_config = self.joint_config['z'][z_joint]
        else:
            raise ValueError(f"z_joint {z_joint} not in joint_config['z']")
        joint_positions_target = self.inverse_kinematics(x,y,z,x_joint_config,y_joint_config,z_joint_config)

        loop_flag = 0


        while loop_flag < len(self.joint_config['joint_names']):
            loop_flag = 0
            for i in range(len(self.joint_config['joint_names'])):
                distance = joint_positions_target[i] - self.j_msg.position[i]
                if distance == 0:
                    loop_flag += 1
                    continue
                minus_flag = distance/abs(distance)
                if abs(distance) > speed/self.rate:
                    self.j_msg.position[i] += minus_flag * speed/self.rate
                else :
                    self.j_msg.position[i] = joint_positions_target[i]
                    loop_flag += 1
                    

            # 发布关节状态
            self.lh_joint_pub_callback()
            time.sleep(1/self.rate)


    def lh_joint_pub_callback(self):
        self.j_msg.header.stamp = self.get_clock().now().to_msg()
        self.j_pub.publish(self.j_msg)

def main():

    pass

if __name__ == '__main__':
    main()