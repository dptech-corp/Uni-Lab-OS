from typing import Any, Dict, Optional
from pylabrobot.resources import Resource as PLRResource
from unilabos.device_comms.modbus_plc.client import ModbusTcpClient
from unilabos.devices.workstation.workstation_base import ResourceSynchronizer, WorkstationBase


class CoinCellAssemblyWorkstation(WorkstationBase):
    def __init__(
        self,
        device_id: str,
        deck_config: Dict[str, Any],
        children: Optional[Dict[str, Any]] = None,
        resource_synchronizer: Optional[ResourceSynchronizer] = None,
        host: str = "192.168.0.0",
        port: str = "",
        *args,
        **kwargs,
    ):
        super().__init__(
            device_id=device_id,
            deck_config=deck_config,
            children=children,
            resource_synchronizer=resource_synchronizer,
            *args,
            **kwargs,
        )
        
        self.hardware_interface = ModbusTcpClient(host=host, port=port)

    def run_assembly(self, wf_name: str, resource: PLRResource, params: str = "\{\}"):
        """启动工作流"""
        self.current_workflow_status = WorkflowStatus.RUNNING
        logger.info(f"工作站 {self.device_id} 启动工作流: {wf_name}")

        # TODO: 实现工作流逻辑

        anode_sheet = self.deck.get_resource("anode_sheet")
        
        