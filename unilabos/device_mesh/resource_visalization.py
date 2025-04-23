from launch import LaunchService
from launch import LaunchDescription
from launch_ros.actions import Node as nd
import xacro
from lxml import etree


class ResourceVisualization:
    def __init__(self, device: dict, registry: dict,  resource: dict, enable_rviz: bool = False):
        
        """初始化资源可视化类
        
        Args:
            device: 设备配置字典
            registry: 注册表字典
        """
        self.launch_service = LaunchService()
        self.launch_description = LaunchDescription()
        self.resource_dict = resource
        self.resource_model = {}
        self.resource_type = ['plate', 'container']

        self.robot_state_str= '''<?xml version="1.0" ?>
        <robot xmlns:xacro="http://ros.org/wiki/xacro" name="full_dev">
        <link name="world"/>
        </robot>
        '''
        self.root = etree.fromstring(self.robot_state_str)
                
        xacro_uri = self.root.nsmap["xacro"]
        # 遍历设备节点
        for node in device['nodes']:
            if node['type'] == 'device':
                device_class = node['class']
                
                # 检查设备类型是否在注册表中
                if device_class not in registry.device_type_registry.keys():
                    raise ValueError(f"设备类型 {device_class} 未在注册表中注册")
                
                elif "model" in device_class.keys():
                    model_config = registry.device_type_registry[device_class]['model']

                    if model_config['type'] == 'device':
                        new_include = etree.SubElement(self.root, f"{{{xacro_uri}}}include")
                        new_include.set("filename", f"{model_config['mesh']}/macro_device.xacro")
                        new_dev = etree.SubElement(self.root, f"{{{xacro_uri}}}{model_config['mesh']}")
                        new_dev.set("parent_link", "world")
                        
            elif node['type'] in self.resource_type:
                resource_class = node['class']
                if resource_class not in registry.resource_type_registry.keys():
                    raise ValueError(f"资源类型 {resource_class} 未在注册表中注册")
                if model_config['type'] == 'resource':
                    model_config = registry.resource_type_registry[resource_class]['model']
                    self.resource_model[node['id']] = model_config['mesh']
                    if model_config['children_mesh'] is not None:
                        self.resource_model[f"{node['id']}_"] = model_config['children_mesh']
        
        re = etree.tostring(self.root, encoding="unicode")
        doc = xacro.parse(re)
        xacro.process_doc(doc)


    def create_launch_description(self, urdf_str: str, enable_rviz: bool = False) -> LaunchDescription:
        """
        创建launch描述，包含robot_state_publisher和move_group节点

        Args:
            urdf_str: URDF文本
            enable_rviz: 是否启用RViz可视化

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

        # 创建move_group节点
        move_group = nd(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'allow_trajectory_execution': True,
                'capabilities': '',
                'disable_capabilities': '',
                'monitor_dynamics': False,
                'publish_monitored_planning_scene': True
            }]
        )

        # 将节点添加到launch描述中
        self.launch_description.add_action(robot_state_publisher)
        self.launch_description.add_action(move_group)

        # 如果启用RViz,添加RViz节点
        if enable_rviz:
            rviz_node = nd(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen'
            )
            self.launch_description.add_action(rviz_node)

        return self.launch_description

    def start(self, urdf_str: str) -> None:
        """
        启动可视化服务

        Args:
            urdf_str: URDF文件路径
        """
        launch_description = self.create_launch_description(urdf_str)
        self.launch_service.include_launch_description(launch_description)
        self.launch_service.run()