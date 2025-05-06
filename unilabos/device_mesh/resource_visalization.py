import os
from pathlib import Path
from launch import LaunchService
from launch import LaunchDescription
from launch_ros.actions import Node as nd
import xacro
from lxml import etree

from unilabos.registry.registry import lab_registry


class ResourceVisualization:
    def __init__(self, device: dict, resource: dict, enable_rviz: bool = True):
        """初始化资源可视化类
        
        该类用于将设备和资源的3D模型可视化展示。通过解析设备和资源的配置信息,
        从注册表中获取对应的3D模型文件,并使用ROS2和RViz进行可视化。
        
        Args:
            device (dict): 设备配置字典,包含设备的类型、位置等信息
            resource (dict): 资源配置字典,包含资源的类型、位置等信息 
            registry (dict): 注册表字典,包含设备和资源类型的注册信息
            enable_rviz (bool, optional): 是否启用RViz可视化. Defaults to True.
        """
        self.launch_service = LaunchService()
        self.launch_description = LaunchDescription()
        self.resource_dict = resource
        self.resource_model = {}
        self.resource_type = ['deck', 'plate', 'container']
        self.mesh_path = Path(__file__).parent.absolute()
        self.enable_rviz = enable_rviz
        registry = lab_registry

        self.srdf_str = '''
        <?xml version="1.0" encoding="UTF-8"?>
        <robot name="minimal">

        </robot>
        '''
        self.robot_state_str= '''<?xml version="1.0" ?>
        <robot xmlns:xacro="http://ros.org/wiki/xacro" name="full_dev">
        <link name="world"/>
        </robot>
        '''
        self.root = etree.fromstring(self.robot_state_str)
                
        xacro_uri = self.root.nsmap["xacro"]

        # 遍历设备节点
        for node in device.values():
            if node['type'] == 'device' and node['class'] != '':
                device_class = node['class']
                # 检查设备类型是否在注册表中
                if device_class not in registry.device_type_registry.keys():
                    raise ValueError(f"设备类型 {device_class} 未在注册表中注册")
            elif node['type'] in self.resource_type:
                # print(registry.resource_type_registry)
                resource_class = node['class']
                if resource_class not in registry.resource_type_registry.keys():
                    raise ValueError(f"资源类型 {resource_class} 未在注册表中注册")
                elif "model" in registry.resource_type_registry[resource_class].keys():
                    model_config = registry.resource_type_registry[resource_class]['model']
                    if model_config['type'] == 'resource':
                        self.resource_model[node['id']] = {
                            'mesh': f"{str(self.mesh_path)}/resources/{model_config['mesh']}",
                            'mesh_tf': model_config['mesh_tf']}
                        if 'children_mesh' in model_config:
                            if model_config['children_mesh'] is not None:
                                self.resource_model[f"{node['id']}_"] = {
                                    'mesh': f"{str(self.mesh_path)}/resources/{model_config['children_mesh']}",
                                    'mesh_tf': model_config['children_mesh_tf']
                                }
                    elif model_config['type'] == 'device':
                        new_include = etree.SubElement(self.root, f"{{{xacro_uri}}}include")
                        new_include.set("filename", f"{str(self.mesh_path)}/devices/{model_config['mesh']}/macro_device.xacro")
                        new_dev = etree.SubElement(self.root, f"{{{xacro_uri}}}{model_config['mesh']}")
                        new_dev.set("parent_link", "world")
                        new_dev.set("mesh_path", str(self.mesh_path))
                        new_dev.set("device_name", node["id"]+"_")
                        new_dev.set("station_name", node["parent"]+'_')
                        new_dev.set("x",str(float(node["position"]["x"])/1000))
                        new_dev.set("y",str(float(node["position"]["y"])/1000))
                        new_dev.set("z",str(float(node["position"]["z"])/1000))
                        if "rotation" in node["config"]:
                            new_dev.set("r",str(float(node["config"]["rotation"]["z"])/1000))
                    else:
                        print("错误的注册表类型！")
        re = etree.tostring(self.root, encoding="unicode")
        doc = xacro.parse(re)
        xacro.process_doc(doc)
        self.urdf_str = doc.toxml()


    def create_launch_description(self, urdf_str: str) -> LaunchDescription:
        """
        创建launch描述，包含robot_state_publisher和move_group节点

        Args:
            urdf_str: URDF文本

        Returns:
            LaunchDescription: launch描述对象
        """

        
        # 解析URDF文件
        robot_description = urdf_str

        # 创建robot_state_publisher节点
        robot_state_publisher = nd(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        )

        # joint_state_publisher_node = nd(
        #     package='joint_state_publisher_gui',  # 或 joint_state_publisher
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher',
        #     output='screen'
        # )
        # 创建move_group节点
        move_group = nd(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'robot_description_semantic': self.srdf_str,
                'capabilities': '',
                'disable_capabilities': '',
                'monitor_dynamics': False,
                'publish_monitored_planning_scene': True,
                'publish_robot_description_semantic': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
            }]
        )

        # 将节点添加到launch描述中
        self.launch_description.add_action(robot_state_publisher)
        # self.launch_description.add_action(joint_state_publisher_node)
        self.launch_description.add_action(move_group)

        # 如果启用RViz,添加RViz节点
        if self.enable_rviz:
            rviz_node = nd(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', f"{str(self.mesh_path)}/view_robot.rviz"],
                output='screen'
            )
            self.launch_description.add_action(rviz_node)

        return self.launch_description

    def start(self) -> None:
        """
        启动可视化服务

        Args:
            urdf_str: URDF文件路径
        """
        launch_description = self.create_launch_description(self.urdf_str)
        self.launch_service.include_launch_description(launch_description)
        self.launch_service.run()