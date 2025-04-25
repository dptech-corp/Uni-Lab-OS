import time
import rclpy,json
from rclpy.node import Node
from std_msgs.msg import String,Header
import numpy as np
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, AllowedCollisionEntry
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.task import Future
import copy
from typing import Tuple, Optional, Union, Any, List
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster, Buffer, TransformListener 
from rclpy.action import ActionServer
from unilabos_msgs.action import SendCmd
from rclpy.action.server import ServerGoalHandle

class ResourceMeshManager(Node):
    def __init__(self, resource_model: dict, resource_config: list, node_name: str):
        """初始化资源网格管理器节点
        
        Args:
            resource_model (dict): 资源模型字典,包含资源的3D模型信息
            resource_config (dict): 资源配置字典,包含资源的配置信息
            node_name (str): 节点名称
        """
        super().__init__(node_name)
        
        self.resource_model = resource_model
        self.resource_config_dict = {item['id']: item for item in resource_config}
        self.move_group_ready = False
        self.resource_tf_dict = {}
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener            = TransformListener(self.tf_buffer, self)
        self.create_timer(0.02, self.publish_resource_tf)

        callback_group = ReentrantCallbackGroup()
        self._get_planning_scene_service = self.create_client(
            srv_type=GetPlanningScene,
            srv_name="get_planning_scene",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=callback_group,
        )
        self.__planning_scene = None
        self.__old_planning_scene = None
        self.__old_allowed_collision_matrix = None
        
        # Create a service for applying the planning scene
        self._apply_planning_scene_service = self.create_client(
            srv_type=ApplyPlanningScene,
            srv_name="apply_planning_scene",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=callback_group,
        )

        self.__collision_object_publisher = self.create_publisher(
            CollisionObject, "/collision_object", 10
        )
        self.__attached_collision_object_publisher = self.create_publisher(
            AttachedCollisionObject, "/attached_collision_object", 10
        )

        # 创建一个Action Server用于修改resource_tf_dict
        self._action_server = ActionServer(
            self,
            SendCmd,
            f'{node_name}/tf_update',
            self.tf_update,
            callback_group=callback_group
        )

    def tf_update(self, goal_handle : ServerGoalHandle):
        tf_update_msg = goal_handle.request
        
        try:
            cmd_dict = json.loads(tf_update_msg.command.replace("'",'"'))
            for resource_id, target_parent in cmd_dict.items():

                # 获取从resource_id到target_parent的转换
                transform = self.tf_buffer.lookup_transform(
                    target_parent,
                    resource_id,
                    rclpy.time.Time()
                )
                
                # 提取转换中的位置和旋转信息
                position = {
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z
                }
                
                rotation = {
                    "x": transform.transform.rotation.x,
                    "y": transform.transform.rotation.y,
                    "z": transform.transform.rotation.z,
                    "w": transform.transform.rotation.w
                }
                
                self.resource_tf_dict[resource_id] = {
                    "parent": target_parent,
                    "position": position,
                    "rotation": rotation
                }

                print(self.resource_tf_dict)
                self.attach_collision_object(id=resource_id,link_name=target_parent)

            self.publish_resource_tf()
            

                
        except Exception as e:
            self.get_logger().error(f"更新资源TF字典失败: {e}")
            goal_handle.abort()
            return SendCmd.Result(success=False)
        goal_handle.succeed()
        return SendCmd.Result(success=True)

    def check_move_group_ready(self):
        """检查move_group节点是否已初始化完成"""
        while not self.move_group_ready:
            # 获取当前可用的节点列表
            if self._get_planning_scene_service.service_is_ready() and self._apply_planning_scene_service.service_is_ready():
                self.resource_mesh_setup()
                self.move_group_ready = True
                
            time.sleep(0.5)
                
    def resource_mesh_setup(self):
        """move_group初始化完成后的设置"""
        self.get_logger().info('开始设置资源网格管理器')
        
        #遍历resource_config中的资源配置，判断panent是否在resource_model中，

        for resource_id, resource_config in self.resource_config_dict.items():

            parent = resource_config['parent']
            parent_link = 'world'
            if parent in self.resource_model:
                parent_link = parent
            elif parent is None and resource_id in self.resource_model:
                pass
            elif parent not in self.resource_model and parent is not None:
                parent_link = f"{self.resource_config_dict[parent]['parent']}{parent}_device_link".replace("None","")
            else:
                continue
            # 提取位置信息并转换单位
            position = {
                "x": float(resource_config['position']['x'])/1000,
                "y": float(resource_config['position']['y'])/1000,
                "z": float(resource_config['position']['z'])/1000
            }
            
            rotation_dict = {
                "x": 0,
                "y": 0,
                "z": 0
            }

            if 'rotation' in resource_config['config']:
                rotation_dict = resource_config['config']['rotation']   
                
            # 从欧拉角转换为四元数
            q = quaternion_from_euler(
                float(rotation_dict['x']), 
                float(rotation_dict['y']), 
                float(rotation_dict['z'])
            )

            rotation = {
                "x": q[0],
                "y": q[1],
                "z": q[2],
                "w": q[3]
            }

            # 更新资源TF字典
            self.resource_tf_dict[resource_id] = {
                "parent": parent_link,
                "position": position,
                "rotation": rotation
            }

    def publish_resource_tf(self):
        """
        发布资源之间的TF关系
        
        遍历self.resource_tf_dict中的每个元素，根据key，parent，以及position和rotation，
        发布key和parent之间的tf关系
        """

        transforms = []
        
        # 遍历资源TF字典
        for resource_id, tf_info in self.resource_tf_dict.items():
            parent = tf_info['parent']
            position = tf_info['position']
            rotation = tf_info['rotation']
            
            # 创建静态变换消息
            
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = parent
            transform.child_frame_id = resource_id
            
            # 设置位置
            transform.transform.translation.x = float(position['x'])
            transform.transform.translation.y = float(position['y'])
            transform.transform.translation.z = float(position['z'])
            
            # 设置旋转
            transform.transform.rotation.x = rotation['x']
            transform.transform.rotation.y = rotation['y']
            transform.transform.rotation.z = rotation['z']
            transform.transform.rotation.w = rotation['w']
            
            transforms.append(transform)
            
        # 一次性发布所有静态变换
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
            # self.get_logger().info(f'已发布 {len(transforms)} 个资源TF关系')

    def add_resource_collision_meshes(self):
        """
        遍历资源配置字典，为每个在resource_model中有对应模型的资源添加碰撞网格
        
        该方法检查每个资源ID是否在self.resource_model中有对应的3D模型文件路径，
        如果有，则调用add_collision_mesh方法将其添加到碰撞环境中。
        """
        self.get_logger().info('开始添加资源碰撞网格')
        
        for resource_id, tf_info in self.resource_tf_dict.items():
            
            if resource_id in self.resource_model:
                # 获取位置信息

                position = [
                    float(self.resource_model[resource_id]['mesh_tf'][0]),
                    float(self.resource_model[resource_id]['mesh_tf'][1]),
                    float(self.resource_model[resource_id]['mesh_tf'][2])
                ]
                
                # 获取旋转信息并转换为四元数

                q = quaternion_from_euler(
                    float(self.resource_model[resource_id]['mesh_tf'][3]), 
                    float(self.resource_model[resource_id]['mesh_tf'][4]), 
                    float(self.resource_model[resource_id]['mesh_tf'][5])
                )
                
                # 添加碰撞网格
                self.add_collision_mesh(
                    filepath=self.resource_model[resource_id]['mesh'],
                    id=resource_id,
                    position=position,
                    quat_xyzw=q,
                    frame_id=resource_id
                )

            elif f"{tf_info['parent']}_" in self.resource_model:
                # 获取资源的父级框架ID
                id_ = f"{tf_info['parent']}_"
                
                # 获取位置信息
                position = [
                    float(self.resource_model[id_]['mesh_tf'][0]),
                    float(self.resource_model[id_]['mesh_tf'][1]),
                    float(self.resource_model[id_]['mesh_tf'][2])
                ]
                
                # 获取旋转信息并转换为四元数

                q = quaternion_from_euler(
                    float(self.resource_model[id_]['mesh_tf'][3]), 
                    float(self.resource_model[id_]['mesh_tf'][4]), 
                    float(self.resource_model[id_]['mesh_tf'][5])
                )
                
                # 添加碰撞网格
                self.add_collision_mesh(
                    filepath=self.resource_model[id_]['mesh'],
                    id=resource_id,
                    position=position,
                    quat_xyzw=q,
                    frame_id=resource_id
                )
            time.sleep(0.01)

        self.get_logger().info('资源碰撞网格添加完成')


    def add_collision_primitive(
        self,
        id: str,
        primitive_type: int,
        dimensions: Tuple[float, float, float],
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a primitive geometry specified by its dimensions.

        `primitive_type` can be one of the following:
            - `SolidPrimitive.BOX`
            - `SolidPrimitive.SPHERE`
            - `SolidPrimitive.CYLINDER`
            - `SolidPrimitive.CONE`
        """

        if (pose is None) and (position is None or quat_xyzw is None):
            raise ValueError(
                "Either `pose` or `position` and `quat_xyzw` must be specified!"
            )

        if isinstance(pose, PoseStamped):
            pose_stamped = pose
        elif isinstance(pose, Pose):
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.__base_link_name
                    ),
                ),
                pose=pose,
            )
        else:
            if not isinstance(position, Point):
                position = Point(
                    x=float(position[0]), y=float(position[1]), z=float(position[2])
                )
            if not isinstance(quat_xyzw, Quaternion):
                quat_xyzw = Quaternion(
                    x=float(quat_xyzw[0]),
                    y=float(quat_xyzw[1]),
                    z=float(quat_xyzw[2]),
                    w=float(quat_xyzw[3]),
                )
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.__base_link_name
                    ),
                ),
                pose=Pose(position=position, orientation=quat_xyzw),
            )

        msg = CollisionObject(
            header=pose_stamped.header,
            id=id,
            operation=operation,
            pose=pose_stamped.pose,
        )

        msg.primitives.append(
            SolidPrimitive(type=primitive_type, dimensions=dimensions)
        )

        self.__collision_object_publisher.publish(msg)

    def add_collision_box(
        self,
        id: str,
        size: Tuple[float, float, float],
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a box geometry specified by its size.
        """

        assert len(size) == 3, "Invalid size of the box!"

        self.add_collision_primitive(
            id=id,
            primitive_type=SolidPrimitive.BOX,
            dimensions=size,
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            operation=operation,
        )

    def add_collision_sphere(
        self,
        id: str,
        radius: float,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a sphere geometry specified by its radius.
        """

        if quat_xyzw is None:
            quat_xyzw = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.add_collision_primitive(
            id=id,
            primitive_type=SolidPrimitive.SPHERE,
            dimensions=[
                radius,
            ],
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            operation=operation,
        )

    def add_collision_cylinder(
        self,
        id: str,
        height: float,
        radius: float,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a cylinder geometry specified by its height and radius.
        """

        self.add_collision_primitive(
            id=id,
            primitive_type=SolidPrimitive.CYLINDER,
            dimensions=[height, radius],
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            operation=operation,
        )

    def add_collision_cone(
        self,
        id: str,
        height: float,
        radius: float,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a cone geometry specified by its height and radius.
        """

        self.add_collision_primitive(
            id=id,
            primitive_type=SolidPrimitive.CONE,
            dimensions=[height, radius],
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            operation=operation,
        )

    def add_collision_mesh(
        self,
        filepath: Optional[str],
        id: str,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
        scale: Union[float, Tuple[float, float, float]] = 1.0,
        mesh: Optional[Any] = None,
    ):
        """
        Add collision object with a mesh geometry. Either `filepath` must be
        specified or `mesh` must be provided.
        Note: This function required 'trimesh' Python module to be installed.
        """

        # Load the mesh
        try:
            import trimesh
        except ImportError as err:
            raise ImportError(
                "Python module 'trimesh' not found! Please install it manually in order "
                "to add collision objects into the MoveIt 2 planning scene."
            ) from err

        # Check the parameters
        if (pose is None) and (position is None or quat_xyzw is None):
            raise ValueError(
                "Either `pose` or `position` and `quat_xyzw` must be specified!"
            )
        if (filepath is None and mesh is None) or (
            filepath is not None and mesh is not None
        ):
            raise ValueError("Exactly one of `filepath` or `mesh` must be specified!")
        if mesh is not None and not isinstance(mesh, trimesh.Trimesh):
            raise ValueError("`mesh` must be an instance of `trimesh.Trimesh`!")

        if isinstance(pose, PoseStamped):
            pose_stamped = pose
        elif isinstance(pose, Pose):
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.__base_link_name
                    ),
                ),
                pose=pose,
            )
        else:
            if not isinstance(position, Point):
                position = Point(
                    x=float(position[0]), y=float(position[1]), z=float(position[2])
                )
            if not isinstance(quat_xyzw, Quaternion):
                quat_xyzw = Quaternion(
                    x=float(quat_xyzw[0]),
                    y=float(quat_xyzw[1]),
                    z=float(quat_xyzw[2]),
                    w=float(quat_xyzw[3]),
                )
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.__base_link_name
                    ),
                ),
                pose=Pose(position=position, orientation=quat_xyzw),
            )

        msg = CollisionObject(
            header=pose_stamped.header,
            id=id,
            operation=operation,
            pose=pose_stamped.pose,
        )

        if filepath is not None:
            mesh = trimesh.load(filepath)

        # Scale the mesh
        if isinstance(scale, float):
            scale = (scale, scale, scale)
        if not (scale[0] == scale[1] == scale[2] == 1.0):
            # If the mesh was passed in as a parameter, make a copy of it to
            # avoid transforming the original.
            if filepath is not None:
                mesh = mesh.copy()
            # Transform the mesh
            transform = np.eye(4)
            np.fill_diagonal(transform, scale)
            mesh.apply_transform(transform)

        msg.meshes.append(
            Mesh(
                triangles=[MeshTriangle(vertex_indices=face) for face in mesh.faces],
                vertices=[
                    Point(x=vert[0], y=vert[1], z=vert[2]) for vert in mesh.vertices
                ],
            )
        )

        self.__collision_object_publisher.publish(msg)

    def remove_collision_object(self, id: str):
        """
        Remove collision object specified by its `id`.
        """

        msg = CollisionObject()
        msg.id = id
        msg.operation = CollisionObject.REMOVE
        msg.header.stamp = self.get_clock().now().to_msg()
        self.__collision_object_publisher.publish(msg)

    def remove_collision_mesh(self, id: str):
        """
        Remove collision mesh specified by its `id`.
        Identical to `remove_collision_object()`.
        """

        self.remove_collision_object(id)

    def attach_collision_object(
        self,
        id: str,
        link_name: Optional[str] = None,
        touch_links: List[str] = [],
        weight: float = 0.0,
    ):
        """
        Attach collision object to the robot.
        """

        if link_name is None:
            link_name = self.__end_effector_name

        msg = AttachedCollisionObject(
            object=CollisionObject(id=id, operation=CollisionObject.ADD)
        )
        msg.link_name = link_name
        msg.touch_links = touch_links
        msg.weight = weight

        self.__attached_collision_object_publisher.publish(msg)

    def detach_collision_object(self, id: int):
        """
        Detach collision object from the robot.
        """

        msg = AttachedCollisionObject(
            object=CollisionObject(id=id, operation=CollisionObject.REMOVE)
        )
        self.__attached_collision_object_publisher.publish(msg)

    def detach_all_collision_objects(self):
        """
        Detach collision object from the robot.
        """

        msg = AttachedCollisionObject(
            object=CollisionObject(operation=CollisionObject.REMOVE)
        )
        self.__attached_collision_object_publisher.publish(msg)

    def move_collision(
        self,
        id: str,
        position: Union[Point, Tuple[float, float, float]],
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        frame_id: Optional[str] = None,
    ):
        """
        Move collision object specified by its `id`.
        """

        msg = CollisionObject()

        if not isinstance(position, Point):
            position = Point(
                x=float(position[0]), y=float(position[1]), z=float(position[2])
            )
        if not isinstance(quat_xyzw, Quaternion):
            quat_xyzw = Quaternion(
                x=float(quat_xyzw[0]),
                y=float(quat_xyzw[1]),
                z=float(quat_xyzw[2]),
                w=float(quat_xyzw[3]),
            )

        pose = Pose()
        pose.position = position
        pose.orientation = quat_xyzw
        msg.pose = pose
        msg.id = id
        msg.operation = CollisionObject.MOVE
        msg.header.frame_id = (
            frame_id if frame_id is not None else self.__base_link_name
        )
        msg.header.stamp = self.get_clock().now().to_msg()

        self.__collision_object_publisher.publish(msg)

    def update_planning_scene(self) -> bool:
        """
        Gets the current planning scene. Returns whether the service call was
        successful.
        """

        if not self._get_planning_scene_service.service_is_ready():
            self.get_logger().warn(
                f"Service '{self._get_planning_scene_service.srv_name}' is not yet available. Better luck next time!"
            )
            return False
        self.__planning_scene = self._get_planning_scene_service.call(
            GetPlanningScene.Request()
        ).scene
        return True

    def allow_collisions(self, id: str, allow: bool) -> Optional[Future]:
        """
        Takes in the ID of an element in the planning scene. Modifies the allowed
        collision matrix to (dis)allow collisions between that object and all other
        object.

        If `allow` is True, a plan will succeed even if the robot collides with that object.
        If `allow` is False, a plan will fail if the robot collides with that object.
        Returns whether it successfully updated the allowed collision matrix.

        Returns the future of the service call.
        """
        # Update the planning scene
        if not self.update_planning_scene():
            return None
        allowed_collision_matrix = self.__planning_scene.allowed_collision_matrix
        self.__old_allowed_collision_matrix = copy.deepcopy(allowed_collision_matrix)

        # Get the location in the allowed collision matrix of the object
        j = None
        if id not in allowed_collision_matrix.entry_names:
            allowed_collision_matrix.entry_names.append(id)
        else:
            j = allowed_collision_matrix.entry_names.index(id)
        # For all other objects, (dis)allow collisions with the object with `id`
        for i in range(len(allowed_collision_matrix.entry_values)):
            if j is None:
                allowed_collision_matrix.entry_values[i].enabled.append(allow)
            elif i != j:
                allowed_collision_matrix.entry_values[i].enabled[j] = allow
        # For the object with `id`, (dis)allow collisions with all other objects
        allowed_collision_entry = AllowedCollisionEntry(
            enabled=[allow for _ in range(len(allowed_collision_matrix.entry_names))]
        )
        if j is None:
            allowed_collision_matrix.entry_values.append(allowed_collision_entry)
        else:
            allowed_collision_matrix.entry_values[j] = allowed_collision_entry

        # Apply the new planning scene
        if not self._apply_planning_scene_service.service_is_ready():
            self.get_logger().warn(
                f"Service '{self._apply_planning_scene_service.srv_name}' is not yet available. Better luck next time!"
            )
            return None
        return self._apply_planning_scene_service.call_async(
            ApplyPlanningScene.Request(scene=self.__planning_scene)
        )

    def process_allow_collision_future(self, future: Future) -> bool:
        """
        Return whether the allow collision service call is done and has succeeded
        or not. If it failed, reset the allowed collision matrix to the old one.
        """
        if not future.done():
            return False

        # Get response
        resp = future.result()

        # If it failed, restore the old planning scene
        if not resp.success:
            self.__planning_scene.allowed_collision_matrix = (
                self.__old_allowed_collision_matrix
            )

        return resp.success

    def clear_all_collision_objects(self) -> Optional[Future]:
        """
        Removes all attached and un-attached collision objects from the planning scene.

        Returns a future for the ApplyPlanningScene service call.
        """
        # Update the planning scene
        if not self.update_planning_scene():
            return None
        self.__old_planning_scene = copy.deepcopy(self.__planning_scene)

        # Remove all collision objects from the planning scene
        self.__planning_scene.world.collision_objects = []
        self.__planning_scene.robot_state.attached_collision_objects = []

        # Apply the new planning scene
        if not self._apply_planning_scene_service.service_is_ready():
            self.get_logger().warn(
                f"Service '{self._apply_planning_scene_service.srv_name}' is not yet available. Better luck next time!"
            )
            return None
        return self._apply_planning_scene_service.call_async(
            ApplyPlanningScene.Request(scene=self.__planning_scene)
        )

if __name__ == '__main__':
    model_s = '''
{'Plate1': {'mesh': '/home/z43/git_pj/Uni-Lab-OS/unilabos/device_mesh/resources/tecan_nested_tip_rack/meshes/plate.stl', 'mesh_tf': [0.064, 0.043, 0, -1.5708, 0, 1.5708]}, 
'Plate1_': {'mesh': '/home/z43/git_pj/Uni-Lab-OS/unilabos/device_mesh/resources/generic_labware_tube_10_75/meshes/0_base.stl', 'mesh_tf': [0.0018, 0.0018, 0, -1.5708,0, 0]}}
'''
    resource_model = json.loads(model_s.replace("'",'"'))

    config_s = '''
[
    {
        "id": "Gripper1",
        "name": "假夹爪",
        "children": [],
        "parent": null,
        "type": "device",
        "class": "gripper.mock",
        "position": {
            "x": 0,
            "y": 0,
            "z": 0
        },
        "config": {},
        "data": {}
    },
    {
        "id": "Plate1",
        "name": "Plate1",
        "sample_id": null,
        "children": [
            "Plate1_A1",
            "Plate1_B1",
            "Plate1_C1",
            "Plate1_D1",
            "Plate1_E1",
            "Plate1_F1",
            "Plate1_G1",
            "Plate1_H1",
            "Plate1_A2",
            "Plate1_B2",
            "Plate1_C2",
            "Plate1_D2",
            "Plate1_E2",
            "Plate1_F2",
            "Plate1_G2",
            "Plate1_H2",
            "Plate1_A3",
            "Plate1_B3",
            "Plate1_C3",
            "Plate1_D3",
            "Plate1_E3",
            "Plate1_F3",
            "Plate1_G3",
            "Plate1_H3",
            "Plate1_A4",
            "Plate1_B4",
            "Plate1_C4",
            "Plate1_D4",
            "Plate1_E4",
            "Plate1_F4",
            "Plate1_G4",
            "Plate1_H4",
            "Plate1_A5",
            "Plate1_B5",
            "Plate1_C5",
            "Plate1_D5",
            "Plate1_E5",
            "Plate1_F5",
            "Plate1_G5",
            "Plate1_H5",
            "Plate1_A6",
            "Plate1_B6",
            "Plate1_C6",
            "Plate1_D6",
            "Plate1_E6",
            "Plate1_F6",
            "Plate1_G6",
            "Plate1_H6",
            "Plate1_A7",
            "Plate1_B7",
            "Plate1_C7",
            "Plate1_D7",
            "Plate1_E7",
            "Plate1_F7",
            "Plate1_G7",
            "Plate1_H7",
            "Plate1_A8",
            "Plate1_B8",
            "Plate1_C8",
            "Plate1_D8",
            "Plate1_E8",
            "Plate1_F8",
            "Plate1_G8",
            "Plate1_H8",
            "Plate1_A9",
            "Plate1_B9",
            "Plate1_C9",
            "Plate1_D9",
            "Plate1_E9",
            "Plate1_F9",
            "Plate1_G9",
            "Plate1_H9",
            "Plate1_A10",
            "Plate1_B10",
            "Plate1_C10",
            "Plate1_D10",
            "Plate1_E10",
            "Plate1_F10",
            "Plate1_G10",
            "Plate1_H10",
            "Plate1_A11",
            "Plate1_B11",
            "Plate1_C11",
            "Plate1_D11",
            "Plate1_E11",
            "Plate1_F11",
            "Plate1_G11",
            "Plate1_H11",
            "Plate1_A12",
            "Plate1_B12",
            "Plate1_C12",
            "Plate1_D12",
            "Plate1_E12",
            "Plate1_F12",
            "Plate1_G12",
            "Plate1_H12"
        ],
        "parent": "Gripper1",
        "type": "device",
        "class": "",
        "position": {
            "x": 0,
            "y": 0,
            "z": 69
        },
        "config": {
            "type": "Plate",
            "size_x": 127.76,
            "size_y": 85.48,
            "size_z": 15.7,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "plate",
            "model": "NEST 96 Well Plate 100 µL PCR Full Skirt",
            "ordering": [
                "A1",
                "B1",
                "C1",
                "D1",
                "E1",
                "F1",
                "G1",
                "H1",
                "A2",
                "B2",
                "C2",
                "D2",
                "E2",
                "F2",
                "G2",
                "H2",
                "A3",
                "B3",
                "C3",
                "D3",
                "E3",
                "F3",
                "G3",
                "H3",
                "A4",
                "B4",
                "C4",
                "D4",
                "E4",
                "F4",
                "G4",
                "H4",
                "A5",
                "B5",
                "C5",
                "D5",
                "E5",
                "F5",
                "G5",
                "H5",
                "A6",
                "B6",
                "C6",
                "D6",
                "E6",
                "F6",
                "G6",
                "H6",
                "A7",
                "B7",
                "C7",
                "D7",
                "E7",
                "F7",
                "G7",
                "H7",
                "A8",
                "B8",
                "C8",
                "D8",
                "E8",
                "F8",
                "G8",
                "H8",
                "A9",
                "B9",
                "C9",
                "D9",
                "E9",
                "F9",
                "G9",
                "H9",
                "A10",
                "B10",
                "C10",
                "D10",
                "E10",
                "F10",
                "G10",
                "H10",
                "A11",
                "B11",
                "C11",
                "D11",
                "E11",
                "F11",
                "G11",
                "H11",
                "A12",
                "B12",
                "C12",
                "D12",
                "E12",
                "F12",
                "G12",
                "H12"
            ]
        },
        "data": {}
    },
    {
        "id": "Plate1_A1",
        "name": "Plate1_A1",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 12.492,
            "y": 72.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_B1",
        "name": "Plate1_B1",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 12.492,
            "y": 63.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_C1",
        "name": "Plate1_C1",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 12.492,
            "y": 54.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_D1",
        "name": "Plate1_D1",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 12.492,
            "y": 45.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_E1",
        "name": "Plate1_E1",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 12.492,
            "y": 36.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_F1",
        "name": "Plate1_F1",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 12.492,
            "y": 27.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_G1",
        "name": "Plate1_G1",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 12.492,
            "y": 18.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_H1",
        "name": "Plate1_H1",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 12.492,
            "y": 9.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_A2",
        "name": "Plate1_A2",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 21.492,
            "y": 72.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_B2",
        "name": "Plate1_B2",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 21.492,
            "y": 63.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_C2",
        "name": "Plate1_C2",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 21.492,
            "y": 54.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_D2",
        "name": "Plate1_D2",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 21.492,
            "y": 45.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_E2",
        "name": "Plate1_E2",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 21.492,
            "y": 36.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_F2",
        "name": "Plate1_F2",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 21.492,
            "y": 27.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_G2",
        "name": "Plate1_G2",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 21.492,
            "y": 18.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_H2",
        "name": "Plate1_H2",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 21.492,
            "y": 9.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_A3",
        "name": "Plate1_A3",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 30.492,
            "y": 72.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_B3",
        "name": "Plate1_B3",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 30.492,
            "y": 63.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_C3",
        "name": "Plate1_C3",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 30.492,
            "y": 54.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_D3",
        "name": "Plate1_D3",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 30.492,
            "y": 45.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_E3",
        "name": "Plate1_E3",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 30.492,
            "y": 36.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_F3",
        "name": "Plate1_F3",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 30.492,
            "y": 27.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_G3",
        "name": "Plate1_G3",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 30.492,
            "y": 18.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_H3",
        "name": "Plate1_H3",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 30.492,
            "y": 9.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_A4",
        "name": "Plate1_A4",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 39.492,
            "y": 72.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_B4",
        "name": "Plate1_B4",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 39.492,
            "y": 63.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_C4",
        "name": "Plate1_C4",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 39.492,
            "y": 54.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_D4",
        "name": "Plate1_D4",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 39.492,
            "y": 45.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_E4",
        "name": "Plate1_E4",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 39.492,
            "y": 36.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_F4",
        "name": "Plate1_F4",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 39.492,
            "y": 27.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_G4",
        "name": "Plate1_G4",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 39.492,
            "y": 18.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_H4",
        "name": "Plate1_H4",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 39.492,
            "y": 9.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_A5",
        "name": "Plate1_A5",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 48.492,
            "y": 72.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_B5",
        "name": "Plate1_B5",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 48.492,
            "y": 63.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_C5",
        "name": "Plate1_C5",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 48.492,
            "y": 54.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_D5",
        "name": "Plate1_D5",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 48.492,
            "y": 45.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_E5",
        "name": "Plate1_E5",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 48.492,
            "y": 36.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_F5",
        "name": "Plate1_F5",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 48.492,
            "y": 27.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_G5",
        "name": "Plate1_G5",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 48.492,
            "y": 18.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_H5",
        "name": "Plate1_H5",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 48.492,
            "y": 9.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_A6",
        "name": "Plate1_A6",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 57.492,
            "y": 72.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_B6",
        "name": "Plate1_B6",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 57.492,
            "y": 63.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_C6",
        "name": "Plate1_C6",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 57.492,
            "y": 54.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_D6",
        "name": "Plate1_D6",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 57.492,
            "y": 45.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_E6",
        "name": "Plate1_E6",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 57.492,
            "y": 36.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_F6",
        "name": "Plate1_F6",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 57.492,
            "y": 27.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_G6",
        "name": "Plate1_G6",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 57.492,
            "y": 18.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_H6",
        "name": "Plate1_H6",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 57.492,
            "y": 9.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_A7",
        "name": "Plate1_A7",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 66.492,
            "y": 72.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_B7",
        "name": "Plate1_B7",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 66.492,
            "y": 63.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_C7",
        "name": "Plate1_C7",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 66.492,
            "y": 54.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_D7",
        "name": "Plate1_D7",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 66.492,
            "y": 45.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_E7",
        "name": "Plate1_E7",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 66.492,
            "y": 36.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_F7",
        "name": "Plate1_F7",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 66.492,
            "y": 27.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_G7",
        "name": "Plate1_G7",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 66.492,
            "y": 18.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_H7",
        "name": "Plate1_H7",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 66.492,
            "y": 9.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_A8",
        "name": "Plate1_A8",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 75.492,
            "y": 72.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_B8",
        "name": "Plate1_B8",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 75.492,
            "y": 63.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_C8",
        "name": "Plate1_C8",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 75.492,
            "y": 54.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_D8",
        "name": "Plate1_D8",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 75.492,
            "y": 45.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_E8",
        "name": "Plate1_E8",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 75.492,
            "y": 36.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_F8",
        "name": "Plate1_F8",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 75.492,
            "y": 27.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_G8",
        "name": "Plate1_G8",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 75.492,
            "y": 18.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_H8",
        "name": "Plate1_H8",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 75.492,
            "y": 9.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_A9",
        "name": "Plate1_A9",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 84.492,
            "y": 72.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_B9",
        "name": "Plate1_B9",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 84.492,
            "y": 63.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_C9",
        "name": "Plate1_C9",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 84.492,
            "y": 54.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_D9",
        "name": "Plate1_D9",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 84.492,
            "y": 45.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_E9",
        "name": "Plate1_E9",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 84.492,
            "y": 36.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_F9",
        "name": "Plate1_F9",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 84.492,
            "y": 27.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_G9",
        "name": "Plate1_G9",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 84.492,
            "y": 18.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_H9",
        "name": "Plate1_H9",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 84.492,
            "y": 9.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_A10",
        "name": "Plate1_A10",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 93.492,
            "y": 72.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_B10",
        "name": "Plate1_B10",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 93.492,
            "y": 63.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_C10",
        "name": "Plate1_C10",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 93.492,
            "y": 54.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_D10",
        "name": "Plate1_D10",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 93.492,
            "y": 45.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_E10",
        "name": "Plate1_E10",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 93.492,
            "y": 36.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_F10",
        "name": "Plate1_F10",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 93.492,
            "y": 27.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_G10",
        "name": "Plate1_G10",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 93.492,
            "y": 18.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_H10",
        "name": "Plate1_H10",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 93.492,
            "y": 9.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_A11",
        "name": "Plate1_A11",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 102.492,
            "y": 72.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_B11",
        "name": "Plate1_B11",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 102.492,
            "y": 63.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_C11",
        "name": "Plate1_C11",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 102.492,
            "y": 54.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_D11",
        "name": "Plate1_D11",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 102.492,
            "y": 45.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_E11",
        "name": "Plate1_E11",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 102.492,
            "y": 36.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_F11",
        "name": "Plate1_F11",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 102.492,
            "y": 27.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_G11",
        "name": "Plate1_G11",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 102.492,
            "y": 18.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_H11",
        "name": "Plate1_H11",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 102.492,
            "y": 9.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_A12",
        "name": "Plate1_A12",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 111.492,
            "y": 72.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_B12",
        "name": "Plate1_B12",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 111.492,
            "y": 63.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_C12",
        "name": "Plate1_C12",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 111.492,
            "y": 54.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_D12",
        "name": "Plate1_D12",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 111.492,
            "y": 45.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_E12",
        "name": "Plate1_E12",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 111.492,
            "y": 36.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_F12",
        "name": "Plate1_F12",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 111.492,
            "y": 27.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_G12",
        "name": "Plate1_G12",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 111.492,
            "y": 18.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    },
    {
        "id": "Plate1_H12",
        "name": "Plate1_H12",
        "sample_id": null,
        "children": [],
        "parent": "Plate1",
        "type": "device",
        "class": "",
        "position": {
            "x": 111.492,
            "y": 9.352,
            "z": 0.92
        },
        "config": {
            "type": "Well",
            "size_x": 3.776,
            "size_y": 3.776,
            "size_z": 14.78,
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "type": "Rotation"
            },
            "category": "well",
            "model": null,
            "max_volume": 100,
            "material_z_thickness": null,
            "compute_volume_from_height": null,
            "compute_height_from_volume": null,
            "bottom_type": "unknown",
            "cross_section_type": "circle"
        },
        "data": {
            "liquids": [],
            "pending_liquids": [],
            "liquid_history": []
        }
    }
]

    '''
    resource_config = json.loads(config_s.replace("'",'"'))
    rclpy.init()
    resource_mesh_manager = ResourceMeshManager(resource_model, resource_config, 'resource_mesh_manager')
    resource_mesh_manager.resource_mesh_setup()
    resource_mesh_manager.publish_resource_tf()
    # resource_mesh_manager.clear_all_collision_objects()
    # print(json.dumps(resource_mesh_manager.resource_tf_dict, indent=4, ensure_ascii=False))
    resource_mesh_manager.add_resource_collision_meshes()
    rclpy.spin(resource_mesh_manager)