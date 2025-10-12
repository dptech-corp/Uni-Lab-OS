      
import json
from typing import List, Optional, Union

from pylabrobot.liquid_handling.backends.backend import (
  LiquidHandlerBackend,
)
from pylabrobot.liquid_handling.standard import (
  Drop,
  DropTipRack,
  MultiHeadAspirationContainer,
  MultiHeadAspirationPlate,
  MultiHeadDispenseContainer,
  MultiHeadDispensePlate,
  Pickup,
  PickupTipRack,
  ResourceDrop,
  ResourceMove,
  ResourcePickup,
  SingleChannelAspiration,
  SingleChannelDispense,
)
from pylabrobot.resources import Resource, Tip

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
from rclpy.action import ActionClient
from unilabos_msgs.action import SendCmd
import re

from unilabos.devices.ros_dev.liquid_handler_joint_publisher import JointStatePublisher


class LiquidHandlerRvizBackend(LiquidHandlerBackend):
  """Chatter box backend for device-free testing. Prints out all operations."""

  _pip_length = 5
  _vol_length = 8
  _resource_length = 20
  _offset_length = 16
  _flow_rate_length = 10
  _blowout_length = 10
  _lld_z_length = 10
  _kwargs_length = 15
  _tip_type_length = 12
  _max_volume_length = 16
  _fitting_depth_length = 20
  _tip_length_length = 16
  # _pickup_method_length = 20
  _filter_length = 10

  def __init__(self, num_channels: int = 8):
    """Initialize a chatter box backend."""
    super().__init__()
    self._num_channels = num_channels
# rclpy.init()
    if not rclpy.ok():
        rclpy.init()
    self.joint_state_publisher = None

  async def setup(self):
    self.joint_state_publisher = JointStatePublisher()
    await super().setup()
  async def stop(self):
    pass

  def serialize(self) -> dict:
    return {**super().serialize(), "num_channels": self.num_channels}

  @property
  def num_channels(self) -> int:
    return self._num_channels

  async def assigned_resource_callback(self, resource: Resource):
    pass

  async def unassigned_resource_callback(self, name: str):
    pass

  async def pick_up_tips(self, ops: List[Pickup], use_channels: List[int], **backend_kwargs):

    for op, channel in zip(ops, use_channels):
      offset = f"{round(op.offset.x, 1)},{round(op.offset.y, 1)},{round(op.offset.z, 1)}"
      row = (
        f"  p{channel}: "
        f"{op.resource.name[-30:]:<{LiquidHandlerRvizBackend._resource_length}} "
        f"{offset:<{LiquidHandlerRvizBackend._offset_length}} "
        f"{op.tip.__class__.__name__:<{LiquidHandlerRvizBackend._tip_type_length}} "
        f"{op.tip.maximal_volume:<{LiquidHandlerRvizBackend._max_volume_length}} "
        f"{op.tip.fitting_depth:<{LiquidHandlerRvizBackend._fitting_depth_length}} "
        f"{op.tip.total_tip_length:<{LiquidHandlerRvizBackend._tip_length_length}} "
        # f"{str(op.tip.pickup_method)[-20:]:<{ChatterboxBackend._pickup_method_length}} "
        f"{'Yes' if op.tip.has_filter else 'No':<{LiquidHandlerRvizBackend._filter_length}}"
      )
    coordinate = ops[0].resource.get_absolute_location(x="c",y="c")
    x = coordinate.x
    y = coordinate.y
    z = coordinate.z + 70
    self.joint_state_publisher.send_resource_action(ops[0].resource.name, x, y, z, "pick")
    #   goback()




  async def drop_tips(self, ops: List[Drop], use_channels: List[int], **backend_kwargs):

    coordinate = ops[0].resource.get_absolute_location(x="c",y="c")
    x = coordinate.x
    y = coordinate.y
    z = coordinate.z + 70
    self.joint_state_publisher.send_resource_action(ops[0].resource.name, x, y, z, "drop_trash")
    #   goback()

  async def aspirate(
    self,
    ops: List[SingleChannelAspiration],
    use_channels: List[int],
    **backend_kwargs,
  ):
    # 执行吸液操作
    pass

    for o, p in zip(ops, use_channels):
      offset = f"{round(o.offset.x, 1)},{round(o.offset.y, 1)},{round(o.offset.z, 1)}"
      row = (
        f"  p{p}: "
        f"{o.volume:<{LiquidHandlerRvizBackend._vol_length}} "
        f"{o.resource.name[-20:]:<{LiquidHandlerRvizBackend._resource_length}} "
        f"{offset:<{LiquidHandlerRvizBackend._offset_length}} "
        f"{str(o.flow_rate):<{LiquidHandlerRvizBackend._flow_rate_length}} "
        f"{str(o.blow_out_air_volume):<{LiquidHandlerRvizBackend._blowout_length}} "
        f"{str(o.liquid_height):<{LiquidHandlerRvizBackend._lld_z_length}} "
        # f"{o.liquids if o.liquids is not None else 'none'}"
      )
      for key, value in backend_kwargs.items():
        if isinstance(value, list) and all(isinstance(v, bool) for v in value):
          value = "".join("T" if v else "F" for v in value)
        if isinstance(value, list):
          value = "".join(map(str, value))
        row += f" {value:<15}"
    coordinate = ops[0].resource.get_absolute_location(x="c",y="c")
    x = coordinate.x
    y = coordinate.y
    z = coordinate.z + 70 
    self.joint_state_publisher.send_resource_action(ops[0].resource.name, x, y, z, "")


  async def dispense(
    self,
    ops: List[SingleChannelDispense],
    use_channels: List[int],
    **backend_kwargs,
  ):

    for o, p in zip(ops, use_channels):
      offset = f"{round(o.offset.x, 1)},{round(o.offset.y, 1)},{round(o.offset.z, 1)}"
      row = (
        f"  p{p}: "
        f"{o.volume:<{LiquidHandlerRvizBackend._vol_length}} "
        f"{o.resource.name[-20:]:<{LiquidHandlerRvizBackend._resource_length}} "
        f"{offset:<{LiquidHandlerRvizBackend._offset_length}} "
        f"{str(o.flow_rate):<{LiquidHandlerRvizBackend._flow_rate_length}} "
        f"{str(o.blow_out_air_volume):<{LiquidHandlerRvizBackend._blowout_length}} "
        f"{str(o.liquid_height):<{LiquidHandlerRvizBackend._lld_z_length}} "
        # f"{o.liquids if o.liquids is not None else 'none'}"
      )
      for key, value in backend_kwargs.items():
        if isinstance(value, list) and all(isinstance(v, bool) for v in value):
          value = "".join("T" if v else "F" for v in value)
        if isinstance(value, list):
          value = "".join(map(str, value))
        row += f" {value:<{LiquidHandlerRvizBackend._kwargs_length}}"
    coordinate = ops[0].resource.get_absolute_location(x="c",y="c")
    x = coordinate.x
    y = coordinate.y
    z = coordinate.z + 70
    self.joint_state_publisher.send_resource_action(ops[0].resource.name, x, y, z, "")

  async def pick_up_tips96(self, pickup: PickupTipRack, **backend_kwargs):
    pass

  async def drop_tips96(self, drop: DropTipRack, **backend_kwargs):
    pass

  async def aspirate96(
    self, aspiration: Union[MultiHeadAspirationPlate, MultiHeadAspirationContainer]
  ):
    pass

  async def dispense96(self, dispense: Union[MultiHeadDispensePlate, MultiHeadDispenseContainer]):
    pass

  async def pick_up_resource(self, pickup: ResourcePickup):
    # 执行资源拾取操作
    pass

  async def move_picked_up_resource(self, move: ResourceMove):
    # 执行资源移动操作
    pass

  async def drop_resource(self, drop: ResourceDrop):
    # 执行资源放置操作
    pass

  def can_pick_up_tip(self, channel_idx: int, tip: Tip) -> bool:
    return True
    