"""
自动化液体处理工作站物料类定义 - 简化版
Automated Liquid Handling Station Resource Classes - Simplified Version
"""

from __future__ import annotations

from typing import Dict, Optional

from pylabrobot.resources.coordinate import Coordinate
from pylabrobot.resources.container import Container
from pylabrobot.resources.resource_holder import ResourceHolder
from pylabrobot.resources import Resource as ResourcePLR


class Bottle(Container):
    """瓶子类 - 简化版，不追踪瓶盖"""

    def __init__(
        self,
        name: str,
        diameter: float,
        height: float,
        max_volume: float,
        size_x: float = 0.0,
        size_y: float = 0.0,
        size_z: float = 0.0,
        barcode: Optional[str] = "",
        category: str = "container",
        model: Optional[str] = None,
    ):
        super().__init__(
            name=name,
            size_x=diameter,
            size_y=diameter,
            size_z=height,
            max_volume=max_volume,
            category=category,
            model=model,
        )
        self.diameter = diameter
        self.height = height
        self.barcode = barcode

    def serialize(self) -> dict:
        return {
            **super().serialize(),
            "diameter": self.diameter,
            "height": self.height,
            "barcode": self.barcode,
        }


from string import ascii_uppercase as LETTERS
from typing import Dict, List, Optional, Type, TypeVar, Union, Sequence, Tuple

import pylabrobot
from pylabrobot.resources.resource_holder import ResourceHolder

T = TypeVar("T", bound=ResourceHolder)

S = TypeVar("S", bound=ResourceHolder)


class ItemizedCarrier(ResourcePLR):
  """Base class for all carriers."""

  def __init__(
    self,
    name: str,
    size_x: float,
    size_y: float,
    size_z: float,
    num_items_x: int = 0,
    num_items_y: int = 0,
    num_items_z: int = 0,
    sites: Optional[Dict[Union[int, str], Optional[ResourcePLR]]] = None,
    category: Optional[str] = "carrier",
    model: Optional[str] = None,
  ):
    super().__init__(
      name=name,
      size_x=size_x,
      size_y=size_y,
      size_z=size_z,
      category=category,
      model=model,
    )
    self.num_items = len(sites)
    self.num_items_x, self.num_items_y, self.num_items_z = num_items_x, num_items_y, num_items_z
    if isinstance(sites, dict):
      sites = sites or {}
      self.sites: List[Optional[ResourcePLR]] = list(sites.values())
      self._ordering = sites
      self.child_locations: Dict[str, Coordinate] = {}
      self.child_size: Dict[str, dict] = {}
      for spot, resource in sites.items():
        if resource is not None and getattr(resource, "location", None) is None:
          raise ValueError(f"resource {resource} has no location")
        if resource is not None:
          self.child_locations[spot] = resource.location
          self.child_size[spot] = {"width": resource._size_x, "height": resource._size_y, "depth": resource._size_z}
        else:
          self.child_locations[spot] = Coordinate.zero()
          self.child_size[spot] = {"width": 0, "height": 0, "depth": 0}
    elif isinstance(sites, list):
      # deserialize时走这里；还需要根据 self.sites 索引children
      self.child_locations = {site["label"]: Coordinate(**site["position"]) for site in sites}
      self.child_size = {site["label"]: site["size"] for site in sites}
      self.sites = [site["occupied_by"] for site in sites]
      self._ordering = {site["label"]: site["position"] for site in sites}
    else:
      print("sites:", sites)

  @property
  def capacity(self):
    """The number of sites on this carrier."""
    return len(self.sites)

  def __len__(self) -> int:
    """Return the number of sites on this carrier."""
    return len(self.sites)

  def assign_child_resource(
    self,
    resource: ResourcePLR,
    location: Optional[Coordinate],
    reassign: bool = True,
    spot: Optional[int] = None,
  ):
    idx = spot
    # 如果只给 location，根据坐标和 deserialize 后的 self.sites（持有names）来寻找 resource 该摆放的位置
    if spot is not None:
      idx = spot
    else:
      for i, site in enumerate(self.sites):
        site_location = list(self.child_locations.values())[i]
        if type(site) == str and site == resource.name:
          idx = i
          break
        if site_location == location:
          idx = i
          break
        
    if not reassign and self.sites[idx] is not None:
      raise ValueError(f"a site with index {idx} already exists")
    super().assign_child_resource(resource, location=location, reassign=reassign)
    self.sites[idx] = resource

  def assign_resource_to_site(self, resource: ResourcePLR, spot: int):
    if self.sites[spot] is not None and not isinstance(self.sites[spot], ResourceHolder):
      raise ValueError(f"spot {spot} already has a resource, {resource}")
    self.assign_child_resource(resource, location=self.child_locations.get(str(spot)), spot=spot)

  def unassign_child_resource(self, resource: ResourcePLR):
    found = False
    for spot, res in enumerate(self.sites):
      if res == resource:
        self.sites[spot] = None
        found = True
        break
    if not found:
      raise ValueError(f"Resource {resource} is not assigned to this carrier")
    if hasattr(resource, "unassign"):
      resource.unassign()

  def __getitem__(
    self,
    identifier: Union[str, int, Sequence[int], Sequence[str], slice, range],
  ) -> Union[List[T], T]:
    """Get the items with the given identifier.

    This is a convenience method for getting the items with the given identifier. It is equivalent
    to :meth:`get_items`, but adds support for slicing and supports single items in the same
    functional call. Note that the return type will always be a list, even if a single item is
    requested.

    Examples:
      Getting the items with identifiers "A1" through "E1":

        >>> items["A1:E1"]

        [<Item A1>, <Item B1>, <Item C1>, <Item D1>, <Item E1>]

      Getting the items with identifiers 0 through 4 (note that this is the same as above):

        >>> items[range(5)]

        [<Item A1>, <Item B1>, <Item C1>, <Item D1>, <Item E1>]

      Getting items with a slice (note that this is the same as above):

        >>> items[0:5]

        [<Item A1>, <Item B1>, <Item C1>, <Item D1>, <Item E1>]

      Getting a single item:

        >>> items[0]

        [<Item A1>]
    """

    if isinstance(identifier, str):
      if ":" in identifier:  # multiple # TODO: deprecate this, use `"A1":"E1"` instead (slice)
        return self.get_items(identifier)

      return self.get_item(identifier)  # single

    if isinstance(identifier, int):
      return self.get_item(identifier)

    if isinstance(identifier, (slice, range)):
      start, stop = identifier.start, identifier.stop
      if isinstance(identifier.start, str):
        start = list(self._ordering.keys()).index(identifier.start)
      elif identifier.start is None:
        start = 0
      if isinstance(identifier.stop, str):
        stop = list(self._ordering.keys()).index(identifier.stop)
      elif identifier.stop is None:
        stop = self.num_items
      identifier = list(range(start, stop, identifier.step or 1))
      return self.get_items(identifier)

    if isinstance(identifier, (list, tuple)):
      return self.get_items(identifier)

    raise TypeError(f"Invalid identifier type: {type(identifier)}")

  def get_item(self, identifier: Union[str, int, Tuple[int, int]]) -> T:
    """Get the item with the given identifier.

    Args:
      identifier: The identifier of the item. Either a string, an integer, or a tuple. If an
      integer, it is the index of the item in the list of items (counted from 0, top to bottom, left
      to right).  If a string, it uses transposed MS Excel style notation, e.g. "A1" for the first
      item, "B1" for the item below that, etc. If a tuple, it is (row, column).

    Raises:
      IndexError: If the identifier is out of range. The range is 0 to self.num_items-1 (inclusive).
    """

    if isinstance(identifier, tuple):
      row, column = identifier
      identifier = LETTERS[row] + str(column + 1)  # standard transposed-Excel style notation
    if isinstance(identifier, str):
      try:
        identifier = list(self._ordering.keys()).index(identifier)
      except ValueError as e:
        raise IndexError(
          f"Item with identifier '{identifier}' does not exist on " f"resource '{self.name}'."
        ) from e

    if not 0 <= identifier < self.capacity:
      raise IndexError(
        f"Item with identifier '{identifier}' does not exist on " f"resource '{self.name}'."
      )

    # Cast child to item type. Children will always be `T`, but the type checker doesn't know that.
    return self.sites[identifier]

  def get_items(self, identifiers: Union[str, Sequence[int], Sequence[str]]) -> List[T]:
    """Get the items with the given identifier.

    Args:
      identifier: Deprecated. Use `identifiers` instead. # TODO(deprecate-ordered-items)
      identifiers: The identifiers of the items. Either a string range or a list of integers. If a
        string, it uses transposed MS Excel style notation. Regions of items can be specified using
        a colon, e.g. "A1:H1" for the first column. If a list of integers, it is the indices of the
        items in the list of items (counted from 0, top to bottom, left to right).

    Examples:
      Getting the items with identifiers "A1" through "E1":

        >>> items.get_items("A1:E1")

        [<Item A1>, <Item B1>, <Item C1>, <Item D1>, <Item E1>]

      Getting the items with identifiers 0 through 4:

        >>> items.get_items(range(5))

        [<Item A1>, <Item B1>, <Item C1>, <Item D1>, <Item E1>]
    """

    if isinstance(identifiers, str):
      identifiers = pylabrobot.utils.expand_string_range(identifiers)
    return [self.get_item(i) for i in identifiers]

  def __setitem__(self, idx: Union[int, str], resource: Optional[ResourcePLR]):
    """Assign a resource to this carrier."""
    if resource is None:  # setting to None
      assigned_resource = self[idx]
      if assigned_resource is not None:
        self.unassign_child_resource(assigned_resource)
    else:
      idx = list(self._ordering.keys()).index(idx) if isinstance(idx, str) else idx
      self.assign_resource_to_site(resource, spot=idx)

  def __delitem__(self, idx: int):
    """Unassign a resource from this carrier."""
    assigned_resource = self[idx]
    if assigned_resource is not None:
      self.unassign_child_resource(assigned_resource)

  def get_resources(self) -> List[ResourcePLR]:
    """Get all resources assigned to this carrier."""
    return [resource for resource in self.sites.values() if resource is not None]

  def __eq__(self, other):
    return super().__eq__(other) and self.sites == other.sites

  def get_free_sites(self) -> List[int]:
    return [spot for spot, resource in self.sites.items() if resource is None]

  def serialize(self):
    return {
      **super().serialize(),
      "num_items_x": self.num_items_x,
      "num_items_y": self.num_items_y,
      "num_items_z": self.num_items_z,
      "sites": [{
        "label": str(identifier),
        "visible": True if self[identifier] is not None else False,
        "occupied_by": self[identifier].name 
                        if isinstance(self[identifier], ResourcePLR) and not isinstance(self[identifier], ResourceHolder) else 
                        self[identifier] if isinstance(self[identifier], str) else None,
        "position": {"x": location.x, "y": location.y, "z": location.z},
        "size": self.child_size[identifier],
        "content_type": ["bottle", "container", "tube", "bottle_carrier", "tip_rack"]
      } for identifier, location in self.child_locations.items()]
    }


class BottleCarrier(ItemizedCarrier):
    """瓶载架 - 直接继承自 TubeCarrier"""

    def __init__(
        self,
        name: str,
        size_x: float,
        size_y: float,
        size_z: float,
        sites: Optional[Dict[Union[int, str], ResourceHolder]] = None,
        category: str = "bottle_carrier",
        model: Optional[str] = None,
    ):
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            sites=sites,
            category=category,
            model=model,
        )
