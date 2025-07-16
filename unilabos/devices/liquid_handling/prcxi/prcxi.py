import asyncio
import collections
import contextlib
import json
import socket
import time
from typing import Any, List, Dict, Optional, TypedDict, Union, Sequence, Iterator, Literal

from pylabrobot.liquid_handling import (
    LiquidHandlerBackend,
    Pickup,
    SingleChannelAspiration,
    Drop,
    SingleChannelDispense,
    PickupTipRack,
    DropTipRack,
    MultiHeadAspirationPlate,
)
from pylabrobot.liquid_handling.standard import (
    MultiHeadAspirationContainer,
    MultiHeadDispenseContainer,
    MultiHeadDispensePlate,
    ResourcePickup,
    ResourceMove,
    ResourceDrop,
)
from pylabrobot.resources import Tip, Deck, Plate, Well, TipRack, Resource, Container, Coordinate, TipSpot, Trash

from unilabos.devices.liquid_handling.liquid_handler_abstract import LiquidHandlerAbstract


class PRCXIError(RuntimeError):
    """Lilith 返回 Success=false 时抛出的业务异常"""


class Material(TypedDict):  # 和Plate同关系
    uuid: str
    Code: Optional[str]
    Name: Optional[str]
    SummaryName: Optional[str]
    PipetteHeight: Optional[int]
    materialEnum: Optional[int]


class WorkTablets(TypedDict):
    Number: int
    Code: str
    Material: Dict[str, Any]


class MatrixInfo(TypedDict):
    MatrixId: str
    MatrixName: str
    MatrixCount: int
    WorkTablets: list[WorkTablets]


class PRCXI9300Deck(Deck):
    """PRCXI 9300 的专用 Deck 类，继承自 Deck。

    该类定义了 PRCXI 9300 的工作台布局和槽位信息。
    """

    def __init__(self, name: str, size_x: float, size_y: float, size_z: float):
        super().__init__(name, size_x, size_y, size_z)
        self.slots = [None] * 6  # PRCXI 9300 有 6 个槽位


class PRCXI9300Container(Plate):
    """PRCXI 9300 的专用 Deck 类，继承自 Deck。

    该类定义了 PRCXI 9300 的工作台布局和槽位信息。
    """

    def __init__(self, name: str, size_x: float, size_y: float, size_z: float, category: str):
        super().__init__(name, size_x, size_y, size_z, category=category, ordering=collections.OrderedDict())
        self._unilabos_state = {}

    def load_state(self, state: Dict[str, Any]) -> None:
        """从给定的状态加载工作台信息。"""
        super().load_state(state)
        self._unilabos_state = state

    def serialize_state(self) -> Dict[str, Dict[str, Any]]:
        data = super().serialize_state()
        data.update(self._unilabos_state)
        return data


class PRCXI9300Handler(LiquidHandlerAbstract):
    @property
    def reset_ok(self) -> bool:
        """检查设备是否已重置成功。"""
        if self._unilabos_backend.debug:
            return True
        return self._unilabos_backend.is_reset_ok

    def __init__(self, deck: Deck, host: str, port: int, timeout: float, setup=True, debug=False):
        tablets_info = []
        count = 0
        for child in deck.children:
            if "Material" in child._unilabos_state:
                count += 1
                tablets_info.append(
                    WorkTablets(Number=count, Code=f"T{count}", Material=child._unilabos_state["Material"])
                )
        self._unilabos_backend = PRCXI9300Backend(tablets_info, host, port, timeout, setup, debug)
        super().__init__(backend=self._unilabos_backend, deck=deck)

    async def create_protocol(
        self,
        protocol_name: str = "",
        protocol_description: str = "",
        protocol_version: str = "",
        protocol_author: str = "",
        protocol_date: str = "",
        protocol_type: str = "",
        none_keys: List[str] = [],
    ):
        self._unilabos_backend.create_protocol(protocol_name)

    async def run_protocol(self):
        return self._unilabos_backend.run_protocol()

    async def remove_liquid(
        self,
        vols: List[float],
        sources: Sequence[Container],
        waste_liquid: Optional[Container] = None,
        *,
        use_channels: Optional[List[int]] = None,
        flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Optional[Literal["wide", "tight", "custom"]] = "wide",
        delays: Optional[List[int]] = None,
        is_96_well: Optional[bool] = False,
        top: Optional[List[float]] = None,
        none_keys: List[str] = [],
    ):
        return await super().remove_liquid(
            vols,
            sources,
            waste_liquid,
            use_channels=use_channels,
            flow_rates=flow_rates,
            offsets=offsets,
            liquid_height=liquid_height,
            blow_out_air_volume=blow_out_air_volume,
            spread=spread,
            delays=delays,
            is_96_well=is_96_well,
            top=top,
            none_keys=none_keys,
        )

    async def add_liquid(
        self,
        asp_vols: Union[List[float], float],
        dis_vols: Union[List[float], float],
        reagent_sources: Sequence[Container],
        targets: Sequence[Container],
        *,
        use_channels: Optional[List[int]] = None,
        flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Optional[Literal["wide", "tight", "custom"]] = "wide",
        is_96_well: bool = False,
        delays: Optional[List[int]] = None,
        mix_time: Optional[int] = None,
        mix_vol: Optional[int] = None,
        mix_rate: Optional[int] = None,
        mix_liquid_height: Optional[float] = None,
        none_keys: List[str] = [],
    ):
        return await super().add_liquid(
            asp_vols,
            dis_vols,
            reagent_sources,
            targets,
            use_channels=use_channels,
            flow_rates=flow_rates,
            offsets=offsets,
            liquid_height=liquid_height,
            blow_out_air_volume=blow_out_air_volume,
            spread=spread,
            is_96_well=is_96_well,
            delays=delays,
            mix_time=mix_time,
            mix_vol=mix_vol,
            mix_rate=mix_rate,
            mix_liquid_height=mix_liquid_height,
            none_keys=none_keys,
        )

    async def transfer_liquid(
        self,
        sources: Sequence[Container],
        targets: Sequence[Container],
        tip_racks: Sequence[TipRack],
        *,
        use_channels: Optional[List[int]] = None,
        asp_vols: Union[List[float], float],
        dis_vols: Union[List[float], float],
        asp_flow_rates: Optional[List[Optional[float]]] = None,
        dis_flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        touch_tip: bool = False,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Literal["wide", "tight", "custom"] = "wide",
        is_96_well: bool = False,
        mix_stage: Optional[Literal["none", "before", "after", "both"]] = "none",
        mix_times: Optional[List[int]] = None,
        mix_vol: Optional[int] = None,
        mix_rate: Optional[int] = None,
        mix_liquid_height: Optional[float] = None,
        delays: Optional[List[int]] = None,
        none_keys: List[str] = [],
    ):
        return await super().transfer_liquid(
            sources,
            targets,
            tip_racks,
            use_channels=use_channels,
            asp_vols=asp_vols,
            dis_vols=dis_vols,
            asp_flow_rates=asp_flow_rates,
            dis_flow_rates=dis_flow_rates,
            offsets=offsets,
            touch_tip=touch_tip,
            liquid_height=liquid_height,
            blow_out_air_volume=blow_out_air_volume,
            spread=spread,
            is_96_well=is_96_well,
            mix_stage=mix_stage,
            mix_times=mix_times,
            mix_vol=mix_vol,
            mix_rate=mix_rate,
            mix_liquid_height=mix_liquid_height,
            delays=delays,
            none_keys=none_keys,
        )

    async def custom_delay(self, seconds=0, msg=None):
        return await super().custom_delay(seconds, msg)

    async def touch_tip(self, targets: Sequence[Container]):
        return await super().touch_tip(targets)

    async def mix(
        self,
        targets: Sequence[Container],
        mix_time: int = None,
        mix_vol: Optional[int] = None,
        height_to_bottom: Optional[float] = None,
        offsets: Optional[Coordinate] = None,
        mix_rate: Optional[float] = None,
        none_keys: List[str] = [],
    ):
        return await self._unilabos_backend.mix(targets, mix_time, mix_vol, height_to_bottom, offsets, mix_rate, none_keys)

    def iter_tips(self, tip_racks: Sequence[TipRack]) -> Iterator[Resource]:
        return super().iter_tips(tip_racks)

    async def pick_up_tips(self, tip_spots: List[TipSpot], use_channels: Optional[List[int]] = None,
                           offsets: Optional[List[Coordinate]] = None, **backend_kwargs):
        return await super().pick_up_tips(tip_spots, use_channels, offsets, **backend_kwargs)

    async def aspirate(self, resources: Sequence[Container], vols: List[float],
                       use_channels: Optional[List[int]] = None, flow_rates: Optional[List[Optional[float]]] = None,
                       offsets: Optional[List[Coordinate]] = None,
                       liquid_height: Optional[List[Optional[float]]] = None,
                       blow_out_air_volume: Optional[List[Optional[float]]] = None,
                       spread: Literal["wide", "tight", "custom"] = "wide", **backend_kwargs):

        return await super().aspirate(resources, vols, use_channels, flow_rates, offsets, liquid_height,
                                      blow_out_air_volume, spread, **backend_kwargs)

    async def drop_tips(self, tip_spots: Sequence[Union[TipSpot, Trash]], use_channels: Optional[List[int]] = None,
                        offsets: Optional[List[Coordinate]] = None, allow_nonzero_volume: bool = False,
                        **backend_kwargs):
        return await super().drop_tips(tip_spots, use_channels, offsets, allow_nonzero_volume, **backend_kwargs)

    async def dispense(self, resources: Sequence[Container], vols: List[float],
                       use_channels: Optional[List[int]] = None, flow_rates: Optional[List[Optional[float]]] = None,
                       offsets: Optional[List[Coordinate]] = None,
                       liquid_height: Optional[List[Optional[float]]] = None,
                       blow_out_air_volume: Optional[List[Optional[float]]] = None,
                       spread: Literal["wide", "tight", "custom"] = "wide", **backend_kwargs):
        return await super().dispense(resources, vols, use_channels, flow_rates, offsets, liquid_height,
                                      blow_out_air_volume, spread, **backend_kwargs)

    async def discard_tips(self, use_channels: Optional[List[int]] = None, allow_nonzero_volume: bool = True,
                           offsets: Optional[List[Coordinate]] = None, **backend_kwargs):
        return await super().discard_tips(use_channels, allow_nonzero_volume, offsets, **backend_kwargs)

    def set_tiprack(self, tip_racks: Sequence[TipRack]):
        super().set_tiprack(tip_racks)

    async def move_to(self, well: Well, dis_to_top: float = 0, channel: int = 0):
        return await super().move_to(well, dis_to_top, channel)


class PRCXI9300Backend(LiquidHandlerBackend):
    """PRCXI 9300 的后端实现，继承自 LiquidHandlerBackend。

    该类提供了与 PRCXI 9300 设备进行通信的基本方法，包括方案管理、自动化控制、运行状态查询等。
    """

    _num_channels = 8  # 默认通道数为 8
    _is_reset_ok = False

    @property
    def is_reset_ok(self) -> bool:
        self._is_reset_ok = self.api_client.get_reset_status()
        return self._is_reset_ok
    
    matrix_info: MatrixInfo
    protocol_name: str
    steps_todo_list = []

    def __init__(
        self,
        tablets_info: list[WorkTablets],
        host: str = "127.0.0.1",
        port: int = 9999,
        timeout: float = 10.0,
        setup=True,
        debug=False,
    ) -> None:
        super().__init__()
        self.tablets_info = tablets_info
        self.api_client = PRCXI9300Api(host, port, timeout, debug)
        self.host, self.port, self.timeout = host, port, timeout
        self._num_channels = 8
        self._execute_setup = setup
        self.debug = debug

    def create_protocol(self, protocol_name):
        self.protocol_name = protocol_name
        self.steps_todo_list = []

    def run_protocol(self):
        assert self.is_reset_ok, "PRCXI9300Backend is not reset successfully. Please call setup() first."
        run_time = time.time()
        self.matrix_info = MatrixInfo(
            MatrixId=f"{int(run_time)}",
            MatrixName=f"protocol_{run_time}",
            MatrixCount=len(self.tablets_info),
            WorkTablets=self.tablets_info,
        )
        #print(json.dumps(self.matrix_info, indent=2))
        res = self.api_client.add_WorkTablet_Matrix(self.matrix_info)
        assert res["Success"], f"Failed to create matrix: {res.get('Message', 'Unknown error')}"
        print(f"PRCXI9300Backend created matrix with ID: {self.matrix_info['MatrixId']}, result: {res}")
        solution_id = self.api_client.add_solution(
            f"protocol_{run_time}", self.matrix_info["MatrixId"], self.steps_todo_list
        )
        print(f"PRCXI9300Backend created solution with ID: {solution_id}")
        self.api_client.load_solution(solution_id)
        return self.api_client.start()

    @classmethod
    def check_channels(cls, use_channels: List[int]) -> List[int]:
        """检查通道是否符合要求，PRCXI9300Backend 只支持所有 8 个通道。"""
        if use_channels != [0, 1, 2, 3, 4, 5, 6, 7]:
            print("PRCXI9300Backend only supports all 8 channels, using default [0, 1, 2, 3, 4, 5, 6, 7].")
            return [0, 1, 2, 3, 4, 5, 6, 7]
        return use_channels

    async def setup(self):
        await super().setup()
        try:
            if self._execute_setup:
                self.api_client.call("IAutomation", "Reset")
                while not self.is_reset_ok:
                    print("Waiting for PRCXI9300 to reset...")
                    await asyncio.sleep(1)
                print("PRCXI9300 reset successfully.")
        except ConnectionRefusedError as e:
            raise RuntimeError(
                f"Failed to connect to PRCXI9300 API at {self.host}:{self.port}. "
                "Please ensure the PRCXI9300 service is running."
            ) from e

    async def stop(self):
        self.api_client.call("IAutomation", "Stop")

    async def pick_up_tips(self, ops: List[Pickup], use_channels: List[int] = None):
        """Pick up tips from the specified resource."""

        if len(ops) != 8:
            raise ValueError(f"PRCXI9300Backend pick_up_tips: Expected 8 pickups, got {len(ops)}")

        plate_indexes = []
        for op in ops:
            plate = op.resource.parent.parent
            deck = plate.parent
            plate_index = deck.children.index(plate)
            plate_indexes.append(plate_index)
        if len(set(plate_indexes)) != 1:
            raise ValueError("All pickups must be from the same plate. Found different plates: " + str(plate_indexes)) 

        tip_columns = []
        for op in ops:
            tipspot = op.resource
            tipspot_index = tipspot.parent.children.index(tipspot)
            tip_columns.append(tipspot_index // 8)
        if len(set(tip_columns)) != 1:
            raise ValueError("All pickups must be from the same tip column. Found different columns: " + str(tip_columns))
        
        PlateNo = plate_indexes[0] + 1
        hole_col = tip_columns[0] + 1

        step = self.api_client.Load(
            "Left",
            dosage=0,
            plate_no=PlateNo,
            is_whole_plate=False,
            hole_row=1,
            hole_col=hole_col,
            blending_times=0,
            balance_height=0,
            plate_or_hole=f"H{hole_col}-8,T{PlateNo}",
            hole_numbers="1,2,3,4,5,6,7,8",
        )
        self.steps_todo_list.append(step)


    async def drop_tips(self, ops: List[Drop], use_channels: List[int] = None):

        """Pick up tips from the specified resource."""

        if len(ops) != 8:
            raise ValueError(f"PRCXI9300Backend drop_tips: Expected 8 pickups, got {len(ops)}")

        plate_indexes = []
        for op in ops:
            plate = op.resource.parent.parent
            deck = plate.parent
            plate_index = deck.children.index(plate)
            plate_indexes.append(plate_index)
        if len(set(plate_indexes)) != 1:
            raise ValueError("All drop_tips must be from the same plate. Found different plates: " + str(plate_indexes)) 

        tip_columns = []
        for op in ops:
            tipspot = op.resource
            tipspot_index = tipspot.parent.children.index(tipspot)
            tip_columns.append(tipspot_index // 8)
        if len(set(tip_columns)) != 1:
            raise ValueError("All drop_tips must be from the same tip column. Found different columns: " + str(tip_columns))
        
        PlateNo = plate_indexes[0] + 1
        hole_col = tip_columns[0] + 1

        step = self.api_client.UnLoad(
            "Left",
            dosage=0,
            plate_no=PlateNo,
            is_whole_plate=False,
            hole_row=1,
            hole_col=hole_col,
            blending_times=0,
            balance_height=0,
            plate_or_hole=f"H{hole_col}-8,T{PlateNo}",
            hole_numbers="1,2,3,4,5,6,7,8",
        )
        self.steps_todo_list.append(step)

    async def mix(
            self,
            targets: Sequence[Container],
            mix_time: int = None,
            mix_vol: Optional[int] = None,
            height_to_bottom: Optional[float] = None,
            offsets: Optional[Coordinate] = None,
            mix_rate: Optional[float] = None,
            none_keys: List[str] = [],
    ):
        
        """Mix liquid in the specified resources."""
   
        if len(targets) != 8:
            raise ValueError(f"PRCXI9300Backend aspirate: Expected 8 aspirate, got {len(targets)}")

        plate_indexes = []
        for op in targets:
            deck = op.parent.parent.parent
            plate = op.parent.parent
            plate_index = deck.children.index(plate)
            plate_indexes.append(plate_index)

        if len(set(plate_indexes)) != 1:
            raise ValueError("All pickups must be from the same plate. Found different plates: " + str(plate_indexes)) 

        tip_columns = []
        for op in targets:
            tipspot_index = op.parent.children.index(op)
            tip_columns.append(tipspot_index // 8)

        if len(set(tip_columns)) != 1:
            raise ValueError("All pickups must be from the same tip column. Found different columns: " + str(tip_columns))
        
        PlateNo = plate_indexes[0] + 1
        hole_col = tip_columns[0] + 1
        assert mix_time > 0
        step = self.api_client.Blending(
            "Left",
            dosage=mix_vol,
            plate_no=PlateNo,
            is_whole_plate=False,
            hole_row=1,
            hole_col=hole_col,
            blending_times=mix_time,
            balance_height=0,
            plate_or_hole=f"H{hole_col}-8,T{PlateNo}",
            hole_numbers="1,2,3,4,5,6,7,8",
        )
        self.steps_todo_list.append(step)

    async def aspirate(self, ops: List[SingleChannelAspiration], use_channels: List[int] = None):

        """Aspirate liquid from the specified resources."""

        if len(ops) != 8:
            raise ValueError(f"PRCXI9300Backend aspirate: Expected 8 aspirate, got {len(ops)}")
        
        plate_indexes = []
        for op in ops:
            plate = op.resource.parent.parent
            deck = plate.parent
            plate_index = deck.children.index(plate)
            plate_indexes.append(plate_index)

        if len(set(plate_indexes)) != 1:
            raise ValueError("All pickups must be from the same plate. Found different plates: " + str(plate_indexes)) 

        tip_columns = []
        for op in ops:
            tipspot = op.resource
            tipspot_index = tipspot.parent.children.index(tipspot)
            tip_columns.append(tipspot_index // 8)

        if len(set(tip_columns)) != 1:
            raise ValueError("All pickups must be from the same tip column. Found different columns: " + str(tip_columns))
        
        volumes = [op.volume for op in ops]
        if len(set(volumes)) != 1:
            raise ValueError("All aspirate volumes must be the same. Found different volumes: " + str(volumes))
        
        PlateNo = plate_indexes[0] + 1
        hole_col = tip_columns[0] + 1

        step = self.api_client.Imbibing(
            "Left",
            dosage=int(volumes[0]),
            plate_no=PlateNo,
            is_whole_plate=False,
            hole_row=1,
            hole_col=hole_col,
            blending_times=0,
            balance_height=0,
            plate_or_hole=f"H{hole_col}-8,T{PlateNo}",
            hole_numbers="1,2,3,4,5,6,7,8",
        )
        self.steps_todo_list.append(step)


    async def dispense(self, ops: List[SingleChannelDispense], use_channels: List[int] = None):
        
        """Dispense liquid into the specified resources."""

        if len(ops) != 8:
            raise ValueError(f"PRCXI9300Backend dispense: Expected 8 dispense, got {len(ops)}")
        
        plate_indexes = []
        for op in ops:
            plate = op.resource.parent.parent
            deck = plate.parent
            plate_index = deck.children.index(plate)
            plate_indexes.append(plate_index)

        if len(set(plate_indexes)) != 1:
            raise ValueError("All dispense must be from the same plate. Found different plates: " + str(plate_indexes)) 

        tip_columns = []
        for op in ops:
            tipspot = op.resource
            tipspot_index = tipspot.parent.children.index(tipspot)
            tip_columns.append(tipspot_index // 8)

        if len(set(tip_columns)) != 1:
            raise ValueError("All dispense must be from the same tip column. Found different columns: " + str(tip_columns))
        
        volumes = [op.volume for op in ops]
        if len(set(volumes)) != 1:
            raise ValueError("All dispense volumes must be the same. Found different volumes: " + str(volumes))
        
        PlateNo = plate_indexes[0] + 1
        hole_col = tip_columns[0] + 1

        step = self.api_client.Tapping(
            "Left",
            dosage=int(volumes[0]),
            plate_no=PlateNo,
            is_whole_plate=False,
            hole_row=1,
            hole_col=hole_col,
            blending_times=0,
            balance_height=0,
            plate_or_hole=f"H{hole_col}-8,T{PlateNo}",
            hole_numbers="1,2,3,4,5,6,7,8",
        )
        self.steps_todo_list.append(step)

    async def pick_up_tips96(self, pickup: PickupTipRack):
        raise NotImplementedError("The PRCXI backend does not support the 96 head.")

    async def drop_tips96(self, drop: DropTipRack):
        raise NotImplementedError("The PRCXI backend does not support the 96 head.")

    async def aspirate96(self, aspiration: Union[MultiHeadAspirationPlate, MultiHeadAspirationContainer]):
        raise NotImplementedError("The Opentrons backend does not support the 96 head.")

    async def dispense96(self, dispense: Union[MultiHeadDispensePlate, MultiHeadDispenseContainer]):
        raise NotImplementedError("The Opentrons backend does not support the 96 head.")

    async def pick_up_resource(self, pickup: ResourcePickup):
        raise NotImplementedError("The Opentrons backend does not support the robotic arm.")

    async def move_picked_up_resource(self, move: ResourceMove):
        raise NotImplementedError("The Opentrons backend does not support the robotic arm.")

    async def drop_resource(self, drop: ResourceDrop):
        raise NotImplementedError("The Opentrons backend does not support the robotic arm.")

    def can_pick_up_tip(self, channel_idx: int, tip: Tip) -> bool:
        return True  # PRCXI9300Backend does not have tip compatibility issues

    def serialize(self) -> dict:
        raise NotImplementedError()

    @property
    def num_channels(self) -> int:
        return self._num_channels


class PRCXI9300Api:
    def __init__(self, host: str = "127.0.0.1", port: int = 9999, timeout: float = 10.0, debug: bool = False) -> None:
        self.host, self.port, self.timeout = host, port, timeout
        self.debug = debug

    @staticmethod
    def _len_prefix(n: int) -> bytes:
        return bytes.fromhex(format(n, "016x"))

    def _raw_request(self, payload: str) -> str:
        if self.debug:
            return " "
        with contextlib.closing(socket.socket()) as sock:
            sock.settimeout(self.timeout)
            sock.connect((self.host, self.port))
            data = payload.encode()
            sock.sendall(self._len_prefix(len(data)) + data)

            chunks, first = [], True
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                if first:
                    chunk, first = chunk[8:], False
                chunks.append(chunk)
            return b"".join(chunks).decode()

    # ---------------------------------------------------- 方案相关（ISolution）
    def list_solutions(self) -> List[Dict[str, Any]]:
        """GetSolutionList"""
        return self.call("ISolution", "GetSolutionList")

    def load_solution(self, solution_id: str) -> bool:
        """LoadSolution"""
        return self.call("ISolution", "LoadSolution", [solution_id])

    def add_solution(self, name: str, matrix_id: str, steps: List[Dict[str, Any]]) -> str:
        """AddSolution → 返回新方案 GUID"""
        return self.call("ISolution", "AddSolution", [name, matrix_id, steps])

    # ---------------------------------------------------- 自动化控制（IAutomation）
    def start(self) -> bool:
        return self.call("IAutomation", "Start")

    def call(self, service: str, method: str, params: Optional[list] = None) -> Any:
        payload = json.dumps(
            {"ServiceName": service, "MethodName": method, "Paramters": params or []}, separators=(",", ":")
        )
        resp = json.loads(self._raw_request(payload))
        if not resp.get("Success", False):
            raise PRCXIError(resp.get("Msg", "Unknown error"))
        data = resp.get("Data")
        try:
            return json.loads(data)
        except (TypeError, json.JSONDecodeError):
            return data

    def pause(self) -> bool:
        """Pause"""
        return self.call("IAutomation", "Pause")

    def resume(self) -> bool:
        """Resume"""
        return self.call("IAutomation", "Resume")

    def get_error_code(self) -> Optional[str]:
        """GetErrorCode"""
        return self.call("IAutomation", "GetErrorCode")

    def get_reset_status(self) -> Optional[str]:
        """GetErrorCode"""
        res = self.call("IAutomation", "GetResetStatus")
        return not res

    def clear_error_code(self) -> bool:
        """RemoveErrorCodet"""
        return self.call("IAutomation", "RemoveErrorCodet")

    # ---------------------------------------------------- 运行状态（IMachineState）
    def step_state_list(self) -> List[Dict[str, Any]]:
        """GetStepStateList"""
        return self.call("IMachineState", "GetStepStateList")

    def step_status(self, seq_num: int) -> Dict[str, Any]:
        """GetStepStatus"""
        return self.call("IMachineState", "GetStepStatus", [seq_num])

    def step_state(self, seq_num: int) -> Dict[str, Any]:
        """GetStepState"""
        return self.call("IMachineState", "GetStepState", [seq_num])

    def axis_location(self, axis_num: int = 1) -> Dict[str, Any]:
        """GetLocation"""
        return self.call("IMachineState", "GetLocation", [axis_num])

    # ---------------------------------------------------- 版位矩阵（IMatrix）
    def list_matrices(self) -> List[Dict[str, Any]]:
        """GetWorkTabletMatrices"""
        return self.call("IMatrix", "GetWorkTabletMatrices")

    def matrix_by_id(self, matrix_id: str) -> Dict[str, Any]:
        """GetWorkTabletMatrixById"""
        return self.call("IMatrix", "GetWorkTabletMatrixById", [matrix_id])

    def add_WorkTablet_Matrix(self, matrix: MatrixInfo):
        return self.call("IMatrix", "AddWorkTabletMatrix", [matrix])

    def Load(
        self,
        axis: str,
        dosage: int,
        plate_no: int,
        is_whole_plate: bool,
        hole_row: int,
        hole_col: int,
        blending_times: int,
        balance_height: int,
        plate_or_hole: str,
        hole_numbers: str,
        assist_fun1: str = "",
        assist_fun2: str = "",
        assist_fun3: str = "",
        assist_fun4: str = "",
        assist_fun5: str = "",
        liquid_method: str = "NormalDispense",
    ) -> Dict[str, Any]:
        return {
            "StepAxis": axis,
            "Function": "Load",
            "DosageNum": dosage,
            "PlateNo": plate_no,
            "IsWholePlate": is_whole_plate,
            "HoleRow": hole_row,
            "HoleCol": hole_col,
            "BlendingTimes": blending_times,
            "BalanceHeight": balance_height,
            "PlateOrHoleNum": plate_or_hole,
            "AssistFun1": assist_fun1,
            "AssistFun2": assist_fun2,
            "AssistFun3": assist_fun3,
            "AssistFun4": assist_fun4,
            "AssistFun5": assist_fun5,
            "HoleNumbers": hole_numbers,
            "LiquidDispensingMethod": liquid_method,
        }

    def Imbibing(
        self,
        axis: str,
        dosage: int,
        plate_no: int,
        is_whole_plate: bool,
        hole_row: int,
        hole_col: int,
        blending_times: int,
        balance_height: int,
        plate_or_hole: str,
        hole_numbers: str,
        assist_fun1: str = "",
        assist_fun2: str = "",
        assist_fun3: str = "",
        assist_fun4: str = "",
        assist_fun5: str = "",
        liquid_method: str = "NormalDispense",
    ) -> Dict[str, Any]:
        return {
            "StepAxis": axis,
            "Function": "Imbibing",
            "DosageNum": dosage,
            "PlateNo": plate_no,
            "IsWholePlate": is_whole_plate,
            "HoleRow": hole_row,
            "HoleCol": hole_col,
            "BlendingTimes": blending_times,
            "BalanceHeight": balance_height,
            "PlateOrHoleNum": plate_or_hole,
            "AssistFun1": assist_fun1,
            "AssistFun2": assist_fun2,
            "AssistFun3": assist_fun3,
            "AssistFun4": assist_fun4,
            "AssistFun5": assist_fun5,
            "HoleNumbers": hole_numbers,
            "LiquidDispensingMethod": liquid_method,
        }

    def Tapping(
        self,
        axis: str,
        dosage: int,
        plate_no: int,
        is_whole_plate: bool,
        hole_row: int,
        hole_col: int,
        blending_times: int,
        balance_height: int,
        plate_or_hole: str,
        hole_numbers: str,
        assist_fun1: str = "",
        assist_fun2: str = "",
        assist_fun3: str = "",
        assist_fun4: str = "",
        assist_fun5: str = "",
        liquid_method: str = "NormalDispense",
    ) -> Dict[str, Any]:
        return {
            "StepAxis": axis,
            "Function": "Tapping",
            "DosageNum": dosage,
            "PlateNo": plate_no,
            "IsWholePlate": is_whole_plate,
            "HoleRow": hole_row,
            "HoleCol": hole_col,
            "BlendingTimes": blending_times,
            "BalanceHeight": balance_height,
            "PlateOrHoleNum": plate_or_hole,
            "AssistFun1": assist_fun1,
            "AssistFun2": assist_fun2,
            "AssistFun3": assist_fun3,
            "AssistFun4": assist_fun4,
            "AssistFun5": assist_fun5,
            "HoleNumbers": hole_numbers,
            "LiquidDispensingMethod": liquid_method,
        }

    def Blending(
        self,
        axis: str,
        dosage: int,
        plate_no: int,
        is_whole_plate: bool,
        hole_row: int,
        hole_col: int,
        blending_times: int,
        balance_height: int,
        plate_or_hole: str,
        hole_numbers: str,
        assist_fun1: str = "",
        assist_fun2: str = "",
        assist_fun3: str = "",
        assist_fun4: str = "",
        assist_fun5: str = "",
        liquid_method: str = "NormalDispense",
    ) -> Dict[str, Any]:
        return {
            "StepAxis": axis,
            "Function": "Blending",
            "DosageNum": dosage,
            "PlateNo": plate_no,
            "IsWholePlate": is_whole_plate,
            "HoleRow": hole_row,
            "HoleCol": hole_col,
            "BlendingTimes": blending_times,
            "BalanceHeight": balance_height,
            "PlateOrHoleNum": plate_or_hole,
            "AssistFun1": assist_fun1,
            "AssistFun2": assist_fun2,
            "AssistFun3": assist_fun3,
            "AssistFun4": assist_fun4,
            "AssistFun5": assist_fun5,
            "HoleNumbers": hole_numbers,
            "LiquidDispensingMethod": liquid_method,
        }

    def UnLoad(
        self,
        axis: str,
        dosage: int,
        plate_no: int,
        is_whole_plate: bool,
        hole_row: int,
        hole_col: int,
        blending_times: int,
        balance_height: int,
        plate_or_hole: str,
        hole_numbers: str,
        assist_fun1: str = "",
        assist_fun2: str = "",
        assist_fun3: str = "",
        assist_fun4: str = "",
        assist_fun5: str = "",
        liquid_method: str = "NormalDispense",
    ) -> Dict[str, Any]:
        return {
            "StepAxis": axis,
            "Function": "UnLoad",
            "DosageNum": dosage,
            "PlateNo": plate_no,
            "IsWholePlate": is_whole_plate,
            "HoleRow": hole_row,
            "HoleCol": hole_col,
            "BlendingTimes": blending_times,
            "BalanceHeight": balance_height,
            "PlateOrHoleNum": plate_or_hole,
            "AssistFun1": assist_fun1,
            "AssistFun2": assist_fun2,
            "AssistFun3": assist_fun3,
            "AssistFun4": assist_fun4,
            "AssistFun5": assist_fun5,
            "HoleNumbers": hole_numbers,
            "LiquidDispensingMethod": liquid_method,
        }


if __name__ == "__main__":
    # Example usage
    deck = PRCXI9300Deck(name="PRCXI Deck", size_x=100, size_y=100, size_z=100)
    plate1 = PRCXI9300Container(name="rackT1", size_x=50, size_y=50, size_z=10, category="plate")
    plate1.load_state({
        "Material": {
            "uuid": "80652665f6a54402b2408d50b40398df",
            "Code": "ZX-001-1000",
            "Name": "1000μL Tip头",
            "SummaryName": "1000μL Tip头",
            "PipetteHeight": 100,
            "materialEnum": 1
        }
    })

    plate2 = PRCXI9300Container(name="plateT2", size_x=50, size_y=50, size_z=10, category="plate")
    plate2.load_state({
        "Material": {
            "uuid": "57b1e4711e9e4a32b529f3132fc5931f",
        }
    })

    plate3 = PRCXI9300Container(name="plateT3", size_x=50, size_y=50, size_z=10, category="plate")
    plate3.load_state({
        "Material": {
            "uuid": "57b1e4711e9e4a32b529f3132fc5931f",
        }
    })

    plate4 = PRCXI9300Container(name="rackT4", size_x=50, size_y=50, size_z=10, category="plate")
    plate4.load_state({
        "Material": {
            "uuid": "80652665f6a54402b2408d50b40398df",
            "Code": "ZX-001-1000",
            "Name": "1000μL Tip头",
            "SummaryName": "1000μL Tip头",
            "PipetteHeight": 100,
            "materialEnum": 1
        }
    })

    plate5 = PRCXI9300Container(name="plateT5", size_x=50, size_y=50, size_z=10, category="plate")
    plate5.load_state({
        "Material": {
            "uuid": "57b1e4711e9e4a32b529f3132fc5931f",
        }
    })
    plate6 = PRCXI9300Container(name="plateT6", size_x=50, size_y=50, size_z=10, category="plate")
    plate6.load_state({
        "Material": {
            "uuid": "57b1e4711e9e4a32b529f3132fc5931f",
        }
    })

    from pylabrobot.resources.opentrons.tip_racks import tipone_96_tiprack_200ul
    from pylabrobot.resources.opentrons.plates import corning_96_wellplate_360ul_flat
    tip_rack = tipone_96_tiprack_200ul("TipRack")
    well_containers = corning_96_wellplate_360ul_flat("Plate")
    # from pprint import pprint
    # pprint(well_containers.children)
    plate1.assign_child_resource(tip_rack, location=Coordinate(0, 0, 0))
    plate2.assign_child_resource(well_containers, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(plate1, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(plate2, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(plate3, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(plate4, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(plate5, location=Coordinate(0, 0, 0))
    deck.assign_child_resource(plate6, location=Coordinate(0, 0, 0))

    handler = PRCXI9300Handler(deck=deck, host="192.168.3.9", port=9999, timeout=10.0, setup=False, debug=True)
    handler.set_tiprack([tip_rack])  # Set the tip rack for the handler
    asyncio.run(handler.setup())  # Initialize the handler and setup the connection
    asyncio.run(handler.create_protocol(protocol_name="Test Protocol"))  # Initialize the backend and setup the connection
    asyncio.run(handler.pick_up_tips(tip_rack.children[:8],[0,1,2,3,4,5,6,7]))
    asyncio.run(handler.aspirate(well_containers.children[:8],[50]*8, [0,1,2,3,4,5,6,7]))
    asyncio.run(handler.dispense(well_containers.children[:8],[50]*8,[0,1,2,3,4,5,6,7]))
    asyncio.run(handler.drop_tips(tip_rack.children[8:16],[0,1,2,3,4,5,6,7]))
    asyncio.run(handler.mix(well_containers.children[:8], mix_time=3, mix_vol=50, height_to_bottom=0.5, offsets=Coordinate(0, 0, 0), mix_rate=100))
    print(json.dumps(handler._unilabos_backend.steps_todo_list, indent=2))  # Print matrix info
    # asyncio.run(handler.add_liquid(
    #     asp_vols=[100]*8,
    #     dis_vols=[100]*8,
    #     reagent_sources=well_containers.children[-8:],
    #     targets=well_containers.children[:8],
    #     use_channels=[0, 1, 2, 3, 4, 5, 6, 7],
    #     flow_rates=[None] * 8,
    #     offsets=[Coordinate(0, 0, 0)] * 8,
    #     liquid_height=[None] * 8,
    #     blow_out_air_volume=[None] * 8,
    #     spread="wide",
    # ))
    # input("pick_up_tips add step")
    asyncio.run(handler.run_protocol())  # Run the protocol
    # input("Running protocol...")
    # input("Press Enter to continue...")  # Wait for user input before proceeding
    # print("PRCXI9300Handler initialized with deck and host settings.")
