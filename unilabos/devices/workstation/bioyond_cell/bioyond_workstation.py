# -*- coding: utf-8 -*-
from typing import Dict, Any, List, Optional
from datetime import datetime, timezone
import requests
from pathlib import Path
import pandas as pd
import time
from datetime import datetime, timezone, timedelta
import re
import threading

from urllib3 import response
from unilabos.devices.workstation.workstation_base import WorkstationBase
from unilabos.devices.workstation.workstation_http_service import WorkstationHTTPService
from unilabos.utils.log import logger
from pylabrobot.resources.deck import Deck

def _iso_utc_now_ms() -> str:
    # 文档要求：到毫秒 + Z，例如 2025-08-15T05:43:22.814Z
    dt = datetime.now()
    return dt.strftime("%Y-%m-%dT%H:%M:%S.") + f"{int(dt.microsecond/1000):03d}Z"


class BioyondWorkstation(WorkstationBase):
    """
    集成 Bioyond LIMS 的工作站示例，
    覆盖：入库(2.17/2.18) → 新建实验(2.14) → 启动调度(2.7) →
    运行中推送：物料变更(2.24)、步骤完成(2.21)、订单完成(2.23) →
    查询实验(2.5/2.6) → 3-2-1 转运(2.32) → 样品/废料取出(2.28)
    """

    def __init__(
        self,
        bioyond_config: Optional[Dict[str, Any]] = None,
        station_resource: Optional[Dict[str, Any]] = None,
        debug_mode: bool = False,
        *args, **kwargs,
    ):
        default_config = {
            #"base_url": "http://192.168.1.200:44388",
            "base_url": "http://61.169.57.196:44422",
            "api_key": "8A819E5C",
            "timeout": 30,
            "report_token": "CHANGE_ME_TOKEN"
        }
        self.bioyond_config = {**default_config, **(bioyond_config or {})}

        self.http_service_started = False
        self.debug_mode = debug_mode
        super().__init__(deck=Deck, station_resource=station_resource, *args, **kwargs)
        logger.info(f"Bioyond工作站初始化完成 (debug_mode={self.debug_mode})")

        # 实例化并在后台线程启动 HTTP 报送服务
    #     self.order_status = {}
    #     try:
    #         t = threading.Thread(target=self._start_http_service_bg, daemon=True, name="unilab_http")
    #         t.start()

    #     except Exception as e:
    #         logger.error(f"unilab-server后台启动报送服务失败: {e}")

    # @property
    # def device_id(self) -> str:
    #     try:
    #         return getattr(self, "_ros_node").device_id  # 兼容 ROS 场景
    #     except Exception:
    #         return "bioyond_workstation"

    # def _start_http_service_bg(self, host: str = "192.168.1.104", port: int = 8080) -> None:
    #     logger.info("进入 _start_http_service_bg 函数")
    #     try:
    #         self.service = WorkstationHTTPService(self, host=host, port=port)
    #         logger.info("WorkstationHTTPService 实例化完成")
    #         self.service.start()
    #         self.http_service_started = True
    #         logger.info(f"unilab_HTTP 服务成功启动: {host}:{port}")

    #         一直挂着，直到进程退出
    #         while True:
    #             time.sleep(1)

    #     except Exception as e:
    #         self.http_service_started = False
    #         logger.error(f"启动unilab_HTTP服务失败: {e}", exc_info=True)

    # -------------------- 基础HTTP封装 --------------------
    def _url(self, path: str) -> str:
        return f"{self.bioyond_config['base_url'].rstrip('/')}/{path.lstrip('/')}"
    def _post_lims(self, path: str, data: Optional[Any] = None) -> Dict[str, Any]:
        """LIMS API：大多数接口用 {apiKey/requestTime,data} 包装"""
        payload = {
            "apiKey": self.bioyond_config["api_key"],
            "requestTime": _iso_utc_now_ms()
        }

        if data is not None:
            payload["data"] = data
      
        if self.debug_mode:
            # 模拟返回，不发真实请求
            logger.info(f"[DEBUG] POST {path} with payload={payload}")
            return {"debug": True, "url": self._url(path), "payload": payload, "status": "ok"}
        try:
            r = requests.post(
                self._url(path),
                json=payload,
                timeout=self.bioyond_config.get("timeout", 30),
                headers={"Content-Type": "application/json"}
            )
            r.raise_for_status()
            #print(r.json())
            return r.json()
        except Exception as e:
            logger.error(f"POST {path} 失败: {e}")
            return {"error": str(e)}

    # --- 修正：_post_report / _post_report_raw 同样走 debug_mode ---
    def _post_report(self, path: str, data: Dict[str, Any]) -> Dict[str, Any]:
        payload = {
            "token": self.bioyond_config.get("report_token", ""),
            "request_time": _iso_utc_now_ms(),
            "data": data
        }
        if self.debug_mode:
            logger.info(f"[DEBUG] POST {path} with payload={payload}")
            return {"debug": True, "url": self._url(path), "payload": payload, "status": "ok"}
        try:
            r = requests.post(self._url(path), json=payload,
                            timeout=self.bioyond_config.get("timeout", 30),
                            headers={"Content-Type": "application/json"})
            r.raise_for_status()
            return r.json()
        except Exception as e:
            logger.error(f"POST {path} 失败: {e}")
            return {"error": str(e)}

    def _post_report_raw(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        if self.debug_mode:
            logger.info(f"[DEBUG] POST {path} with body={body}")
            return {"debug": True, "url": self._url(path), "payload": body, "status": "ok"}
        try:
            r = requests.post(self._url(path), json=body,
                            timeout=self.bioyond_config.get("timeout", 30),
                            headers={"Content-Type": "application/json"})
            r.raise_for_status()
            return r.json()
        except Exception as e:
            logger.error(f"POST {path} 失败: {e}")
            return {"error": str(e)}


    # -------------------- 单点接口封装 --------------------
    # 2.17 入库物料（单个）
    def storage_inbound(self, material_id: str, location_id: str) -> Dict[str, Any]:
        return self._post_lims("/api/lims/storage/inbound", {
            "materialId": material_id,
            "locationId": location_id
        })

    # 2.18 批量入库（多个）
    def storage_batch_inbound(self, items: List[Dict[str, str]]) -> Dict[str, Any]:
        """
        items = [{"materialId": "...", "locationId": "..."}, ...]
        """
        return self._post_lims("/api/lims/storage/batch-inbound", items)

    # 3.30 自动化上料（Excel -> JSON -> POST /api/lims/order/auto-feeding4to3）
    def auto_feeding4to3_from_xlsx(self, xlsx_path: str) -> Dict[str, Any]:
        """
        根据固定模板解析 Excel：
        - 四号手套箱加样头面 (2-13行, 3-7列)
        - 四号手套箱原液瓶面 (15-23行, 3-9列)
        - 三号手套箱人工堆栈 (26-40行, 3-7列)
        """
        path = Path(xlsx_path)
        if not path.exists():
            raise FileNotFoundError(f"未找到 Excel 文件：{path}")

        try:
            df = pd.read_excel(path, sheet_name=0, header=None, engine="openpyxl")
        except Exception as e:
            raise RuntimeError(f"读取 Excel 失败：{e}")

        items: List[Dict[str, Any]] = []

        # 四号手套箱 - 加样头面（2-13行, 3-7列）
        for _, row in df.iloc[1:13, 2:7].iterrows():
            item = {
                "sourceWHName": "四号手套箱堆栈",
                "posX": int(row[2]),
                "posY": int(row[3]),
                "posZ": int(row[4]),
                "materialName": str(row[5]).strip() if pd.notna(row[5]) else "",
                "quantity": float(row[6]) if pd.notna(row[6]) else 0.0,
            }
            if item["materialName"]:
                items.append(item)

        # 四号手套箱 - 原液瓶面（15-23行, 3-9列）
        for _, row in df.iloc[14:23, 2:9].iterrows():
            item = {
                "sourceWHName": "四号手套箱堆栈",
                "posX": int(row[2]),
                "posY": int(row[3]),
                "posZ": int(row[4]),
                "materialName": str(row[5]).strip() if pd.notna(row[5]) else "",
                "quantity": float(row[6]) if pd.notna(row[6]) else 0.0,
                "materialType": str(row[7]).strip() if pd.notna(row[7]) else "",
                "targetWH": str(row[8]).strip() if pd.notna(row[8]) else "",
            }
            if item["materialName"]:
                items.append(item)

        # 三号手套箱人工堆栈（26-40行, 3-7列）
        for _, row in df.iloc[25:40, 2:7].iterrows():
            item = {
                "sourceWHName": "三号手套箱人工堆栈",
                "posX": int(row[2]),
                "posY": int(row[3]),
                "posZ": int(row[4]),
                "materialType": str(row[5]).strip() if pd.notna(row[5]) else "",
                "materialId": str(row[6]).strip() if pd.notna(row[6]) else "",
                "quantity": 1  # 默认数量1
            }
            if item["materialId"] or item["materialType"]:
                items.append(item)

        return self._post_lims("/api/lims/order/auto-feeding4to3", items)



    def auto_batch_outbound_from_xlsx(self, xlsx_path: str) -> Dict[str, Any]:
        """
        3.31 自动化下料（Excel -> JSON -> POST /api/lims/storage/auto-batch-out-bound）
        """
        path = Path(xlsx_path)
        if not path.exists():
            raise FileNotFoundError(f"未找到 Excel 文件：{path}")

        try:
            df = pd.read_excel(path, sheet_name=0, engine="openpyxl")
        except Exception as e:
            raise RuntimeError(f"读取 Excel 失败：{e}")

        def pick(names: List[str]) -> Optional[str]:
            for n in names:
                if n in df.columns:
                    return n
            return None

        c_loc = pick(["locationId", "库位ID", "库位Id", "库位id"])
        c_wh  = pick(["warehouseId", "仓库ID", "仓库Id", "仓库id"])
        c_qty = pick(["数量", "quantity"])
        c_x   = pick(["x", "X", "posX", "坐标X"])
        c_y   = pick(["y", "Y", "posY", "坐标Y"])
        c_z   = pick(["z", "Z", "posZ", "坐标Z"])

        required = [c_loc, c_wh, c_qty, c_x, c_y, c_z]
        if any(c is None for c in required):
            raise KeyError("Excel 缺少必要列：locationId/warehouseId/数量/x/y/z（支持多别名，至少要能匹配到）。")

        def as_int(v, d=0):
            try:
                if pd.isna(v): return d
                return int(v)
            except Exception:
                try:
                    return int(float(v))
                except Exception:
                    return d

        def as_float(v, d=0.0):
            try:
                if pd.isna(v): return d
                return float(v)
            except Exception:
                return d

        def as_str(v, d=""):
            if v is None or (isinstance(v, float) and pd.isna(v)): return d
            s = str(v).strip()
            return s if s else d

        items: List[Dict[str, Any]] = []
        for _, row in df.iterrows():
            items.append({
                "locationId": as_str(row[c_loc]),
                "warehouseId": as_str(row[c_wh]),
                "quantity": as_float(row[c_qty]),
                "x": as_int(row[c_x]),
                "y": as_int(row[c_y]),
                "z": as_int(row[c_z]),
            })

        return self._post_lims("/api/lims/storage/auto-batch-out-bound", items)

    # 2.14 新建实验
    def create_orders(self, xlsx_path: str) -> Dict[str, Any]:
        """
        从 Excel 解析并创建实验（2.14）
        约定：
        - batchId = Excel 文件名（不含扩展名）
        - 物料列：所有以 "(g)" 结尾（不再读取“总质量(g)”列）
        - totalMass 自动计算为所有物料质量之和
        - createTime 缺失或为空时自动填充为当前日期（YYYY/M/D）
        """
        path = Path(xlsx_path)
        if not path.exists():
            raise FileNotFoundError(f"未找到 Excel 文件：{path}")

        try:
            df = pd.read_excel(path, sheet_name=0, engine="openpyxl")
        except Exception as e:
            raise RuntimeError(f"读取 Excel 失败：{e}")

        # 列名容错：返回可选列名，找不到则返回 None
        def _pick(col_names: List[str]) -> Optional[str]:
            for c in col_names:
                if c in df.columns:
                    return c
            return None

        col_order_name = _pick(["配方ID", "orderName", "订单编号"])
        col_create_time = _pick(["创建日期", "createTime"])
        col_bottle_type = _pick(["配液瓶类型", "bottleType"])
        col_mix_time = _pick(["混匀时间(s)", "mixTime"])
        col_load = _pick(["扣电组装分液体积", "loadSheddingInfo"])
        col_pouch = _pick(["软包组装分液体积", "pouchCellInfo"])
        col_cond = _pick(["电导测试分液体积", "conductivityInfo"])
        col_cond_cnt = _pick(["电导测试分液瓶数", "conductivityBottleCount"])

        # 物料列：所有以 (g) 结尾
        material_cols = [c for c in df.columns if isinstance(c, str) and c.endswith("(g)")]
        if not material_cols:
            raise KeyError("未发现任何以“(g)”结尾的物料列，请检查表头。")

        batch_id = path.stem

        def _to_ymd_slash(v) -> str:
            # 统一为 "YYYY/M/D"；为空或解析失败则用当前日期
            if v is None or (isinstance(v, float) and pd.isna(v)) or str(v).strip() == "":
                ts = datetime.now()
            else:
                try:
                    ts = pd.to_datetime(v)
                except Exception:
                    ts = datetime.now()
            return f"{ts.year}/{ts.month}/{ts.day}"

        def _as_int(val, default=0) -> int:
            try:
                if pd.isna(val):
                    return default
                return int(val)
            except Exception:
                return default

        def _as_str(val, default="") -> str:
            if val is None or (isinstance(val, float) and pd.isna(val)):
                return default
            s = str(val).strip()
            return s if s else default

        orders: List[Dict[str, Any]] = []

        for idx, row in df.iterrows():
            mats: List[Dict[str, Any]] = []
            total_mass = 0.0

            for mcol in material_cols:
                val = row.get(mcol, None)
                if val is None or (isinstance(val, float) and pd.isna(val)):
                    continue
                try:
                    mass = float(val)
                except Exception:
                    continue
                if mass > 0:
                    mats.append({"name": mcol.replace("(g)", ""), "mass": mass})
                    total_mass += mass

            order_data = {
                "batchId": batch_id,
                "orderName": _as_str(row[col_order_name], default=f"{batch_id}_order_{idx+1}") if col_order_name else f"{batch_id}_order_{idx+1}",
                "createTime": _to_ymd_slash(row[col_create_time]) if col_create_time else _to_ymd_slash(None),
                "bottleType": _as_str(row[col_bottle_type], default="配液小瓶") if col_bottle_type else "配液小瓶",
                "mixTime": _as_int(row[col_mix_time]) if col_mix_time else 0,
                "loadSheddingInfo": _as_int(row[col_load]) if col_load else 0,
                "pouchCellInfo": _as_int(row[col_pouch]) if col_pouch else 0,
                "conductivityInfo": _as_int(row[col_cond]) if col_cond else 0,
                "conductivityBottleCount": _as_int(row[col_cond_cnt]) if col_cond_cnt else 0,
                "materialInfos": mats,
                "totalMass": round(total_mass, 4)  # 自动汇总
            }
            orders.append(order_data)

        # print(orders)
        while True:
            time.sleep(5)
            response = self._post_lims("/api/lims/order/orders", orders)
            if response.get("data", []):
                break
            logger.info(f"等待配液实验创建完成")


       
        # self.order_status[response["data"]["orderCode"]] = "running"

        # while True:
        #     time.sleep(5)
        #     if self.order_status.get(response["data"]["orderCode"], None) == "finished":
        #         logger.info(f"配液实验已完成 ，即将执行 3-2-1 转运")
        #         break
        #     logger.info(f"等待配液实验完成")

        # self.transfer_3_to_2_to_1()
        # self.wait_for_transfer_task()
        # logger.info(f"3-2-1 转运完成，返回结果")
        # return r321
        return response

    # 2.7 启动调度
    def scheduler_start(self) -> Dict[str, Any]:
        response = self._post_lims("/api/lims/scheduler/start")
        print(response)
        return response
    # 3.10 停止调度
    def scheduler_stop(self) -> Dict[str, Any]:
        """
        停止调度 (3.10)
        请求体只包含 apiKey 和 requestTime
        """
        return self._post_lims("/api/lims/scheduler/stop")
            # 2.9 继续调度
    def scheduler_continue(self) -> Dict[str, Any]:
        """
        继续调度 (2.9)
        请求体只包含 apiKey 和 requestTime
        """
        return self._post_lims("/api/lims/scheduler/continue")



    # 2.24 物料变更推送
    def report_material_change(self, material_obj: Dict[str, Any]) -> Dict[str, Any]:
        """
        material_obj 按 2.24 的裸对象格式（包含 id/typeName/locations/detail 等）
        """
        return self._post_report_raw("/report/material_change", material_obj)

    # 2.21 步骤完成推送（BS → LIMS）
    def report_step_finish(self,
                           order_code: str,
                           order_name: str,
                           step_name: str,
                           step_id: str,
                           sample_id: str,
                           start_time: str,
                           end_time: str,
                           execution_status: str = "completed") -> Dict[str, Any]:
        data = {
            "orderCode": order_code,
            "orderName": order_name,
            "stepName": step_name,
            "stepId": step_id,
            "sampleId": sample_id,
            "startTime": start_time,
            "endTime": end_time,
            "executionStatus": execution_status
        }
        return self._post_report("/report/step_finish", data)

    # 2.23 订单完成推送（BS → LIMS）
    def report_order_finish(self,
                            order_code: str,
                            order_name: str,
                            start_time: str,
                            end_time: str,
                            status: str = "30",  # 30 完成 / -11 异常停止 / -12 人工停止
                            workflow_status: str = "Finished",
                            completion_time: Optional[str] = None,
                            used_materials: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
        data = {
            "orderCode": order_code,
            "orderName": order_name,
            "startTime": start_time,
            "endTime": end_time,
            "status": status,
            "workflowStatus": workflow_status,
            "completionTime": completion_time or end_time,
            "usedMaterials": used_materials or []
        }
        return self._post_report("/report/order_finish", data)

    # 2.5 批量查询实验报告（用于轮询是否完成）
    def order_list(self,
                   status: Optional[str] = None,
                   begin_time: Optional[str] = None,
                   end_time: Optional[str] = None,
                   filter_text: Optional[str] = None,
                   skip: int = 0, page: int = 10) -> Dict[str, Any]:
        data: Dict[str, Any] = {"skipCount": skip, "pageCount": page}
        if status is not None:  # 80 成功 / 90 失败 / 100 执行中
            data["status"] = status
        if begin_time:
            data["timeType"] = "CreationTime"
            data["beginTime"] = begin_time
        if end_time:
            data["endTime"] = end_time
        if filter_text:
            data["filter"] = filter_text
        return self._post_lims("/api/lims/order/order-list", data)

    # 2.6 实验报告查询（根据任务ID拿详情）
    def order_report(self, order_id: str) -> Dict[str, Any]:
        return self._post_lims("/api/lims/order/order-report", order_id)

    # 2.32 3-2-1 物料转运
    def transfer_3_to_2_to_1(self,
                            #  source_wh_id: Optional[str] = None,
                            source_wh_id: Optional[str] = '3a19debc-84b4-0359-e2d4-b3beea49348b',
                             source_x: int = 1, source_y: int = 1, source_z: int = 1) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "sourcePosX": source_x, "sourcePosY": source_y, "sourcePosZ": source_z
        }
        if source_wh_id:
            payload["sourceWHID"] = source_wh_id
        return self._post_lims("/api/lims/order/transfer-task3To2To1", payload)

    # 2.28 样品/废料取出
    def take_out(self,
                 order_id: str,
                 preintake_ids: Optional[List[str]] = None,
                 material_ids: Optional[List[str]] = None) -> Dict[str, Any]:
        data = {
            "orderId": order_id,
            "preintakeIds": preintake_ids or [],
            "materialIds": material_ids or []
        }
        return self._post_lims("/api/lims/order/take-out", data)

    # --------（可选）占位方法：文档未定义的“1号站内部流程 / 1-2转运”--------
    def start_station1_internal_flow(self, **kwargs) -> None:
        logger.info("启动1号站内部流程（占位，按现场系统填充具体指令）")


    # 3.x 1→2 物料转运
    def transfer_1_to_2(self) -> Dict[str, Any]:
        """
        1→2 物料转运
        URL: /api/lims/order/transfer-task1To2
        只需要 apiKey 和 requestTime
        """
        return self._post_lims("/api/lims/order/transfer-task1To2")


    # -------------------- 整体编排 --------------------
    def run_full_workflow(self,
                          inbound_items: List[Dict[str, str]],
                          orders: List[Dict[str, Any]],
                          poll_filter_code: Optional[str] = None,
                          poll_timeout_s: int = 600,
                          poll_interval_s: int = 5,
                          transfer_source: Optional[Dict[str, Any]] = None,
                          takeout_order_id: Optional[str] = None) -> None:
        """
        一键串联：
        1) 入库 3-4 个物料 → 2) 新建实验 → 3) 启动调度
        运行中（如需）：4) 物料变更推送 5) 步骤完成推送 6) 订单完成推送
        完成后：查询实验（2.5/2.6）→ 7) 3-2-1 转运 → 8) 1号站内部流程
        → 9) 1-2 转运 → 10) 样品/废料取出
        """
        # 1. 入库（多于1个就用批量接口 2.18）
        if len(inbound_items) == 1:
            r = self.storage_inbound(inbound_items[0]["materialId"], inbound_items[0]["locationId"])
            logger.info(f"单个入库结果: {r}")
        else:
            r = self.storage_batch_inbound(inbound_items)
            logger.info(f"批量入库结果: {r}")

        # 2. 新建实验（2.14）
        r = self.create_orders(orders)
        logger.info(f"新建实验结果: {r}")

        # 3. 启动调度（2.7）
        r = self.scheduler_start()
        logger.info(f"启动调度结果: {r}")

        # —— 运行中各类推送（2.24 / 2.21 / 2.23），通常由实际任务驱动，这里提供调用方式 —— #
        # self.report_material_change({...})
        # self.report_step_finish(order_code="BSO...", order_name="配液分液", step_name="xxx", step_id="...", sample_id="...",
        #                         start_time=_iso_utc_now_ms(), end_time=_iso_utc_now_ms(), execution_status="completed")
        # self.report_order_finish(order_code="BSO...", order_name="配液分液", start_time="...", end_time=_iso_utc_now_ms())

        # 完成后才能转运：用 2.5 批量查询配合 filter=任务编码 轮询到 status=80（成功）
        if poll_filter_code:
            import time
            deadline = time.time() + poll_timeout_s
            while time.time() < deadline:
                res = self.order_list(status="80", filter_text=poll_filter_code, page=5)
                if isinstance(res, dict) and res.get("data", {}).get("items"):
                    logger.info(f"实验 {poll_filter_code} 已完成：{res['data']['items'][0]}")
                    break
                time.sleep(poll_interval_s)
            else:
                logger.warning(f"等待实验 {poll_filter_code} 完成超时（未到 status=80）")

        # 7. 启动 3-2-1 转运（2.32）
        if transfer_source:
            r = self.transfer_3_to_2_to_1(
                source_wh_id=transfer_source.get("sourceWHID"),
                source_x=transfer_source.get("sourcePosX", 1),
                source_y=transfer_source.get("sourcePosY", 1),
                source_z=transfer_source.get("sourcePosZ", 1),
            )
            logger.info(f"3-2-1 转运结果: {r}")

        # 8. 1号站内部流程（占位）
        self.start_station1_internal_flow()

        # 9. 1→2 转运（占位）
        self.transfer_1_to_2()

        # 10. 样品/废料取出（2.28）
        if takeout_order_id:
            r = self.take_out(order_id=takeout_order_id)
            logger.info(f"样品/废料取出结果: {r}")

    # 2.5 批量查询实验报告
    def order_list_v2(self,
                      timeType: str = "string",
                      beginTime: str = "",
                      endTime: str = "",
                      status: str = "",
                      filter: str = "",
                      skipCount: int = 0,
                      pageCount: int = 1,
                      sorting: str = "") -> Dict[str, Any]:
        """
        批量查询实验报告的详细信息 (2.5)
        URL: /api/lims/order/order-list
        参数默认值和接口文档保持一致
        """
        data: Dict[str, Any] = {
            "timeType": timeType,
            "beginTime": beginTime,
            "endTime": endTime,
            "status": status,
            "filter": filter,
            "skipCount": skipCount,
            "pageCount": pageCount,
            "sorting": sorting
        }
        return self._post_lims("/api/lims/order/order-list", data)

        
    def wait_for_transfer_task(self, timeout: int = 3000, interval: int = 5, filter_text: Optional[str] = None) -> bool:
        """
        轮询查询物料转移任务是否成功完成 (status=80)
        - timeout: 最大等待秒数 (默认600秒)
        - interval: 轮询间隔秒数 (默认3秒)
        返回 True 表示找到并成功完成，False 表示超时未找到
        """
        now = datetime.now()
        beginTime = now.strftime("%Y-%m-%dT%H:%M:%SZ")
        endTime = (now + timedelta(minutes=5)).strftime("%Y-%m-%dT%H:%M:%SZ")
        print(beginTime, endTime)

        deadline = time.time() + timeout

        while time.time() < deadline:
            result = self.order_list_v2(
                timeType="string",
                beginTime=beginTime,
                endTime=endTime,
                status="",
                filter=filter_text,
                skipCount=0,
                pageCount=1,
                sorting=""
            )
            print(result)

            items = result.get("data", {}).get("items", [])
            for item in items:
                name = item.get("name", "")
                status = item.get("status")
                # 改成用 filter_text 判断
                if (not filter_text or filter_text in name) and status == 80:
                    logger.info(f"硬件转移动作完成: {name}, status={status}")
                    return True

                logger.info(f"等待中: {name}, status={status}")
            time.sleep(interval)

        logger.warning("超时未找到成功的物料转移任务")
        return False
    

    def Bioystation_scheduler_start_task(self) -> bool:
        logger.info("开始调度")
        self.scheduler_start()
        logger.info("调度已启动")

    def Bioystation_scheduler_stop_task(self) -> bool:
        logger.info("停止调度")
        self.scheduler_stop()
        logger.info("调度已停止")

    def Bioystation_scheduler_continue_task(self) -> bool:
        logger.info("继续调度")
        self.scheduler_continue()
        logger.info("调度已继续")

    # 3.30 上料：读取模板 Excel 自动解析并 POST
    def Bioystation_feeding4to3_from_xlsx_task(self) -> bool:
        logger.info("4号箱自动上料开始")
        r1 = self.auto_feeding4to3_from_xlsx(r"D:\Uni-lab\Uni-Lab-OS\unilabos\devices\workstation\bioyond_cell\样品导入模板.xlsx")
        self.wait_for_transfer_task(filter_text="物料转移任务")
        logger.info("4号箱向3号箱转运物料转移任务已完成")
        return True
    
    # # 新建实验
    def Bioystation_start_experiment_task(self) -> bool:
        logger.info("3号箱内实验开始")
        response = self.create_orders(r"D:\Uni-lab\Uni-Lab-OS\unilabos\devices\workstation\bioyond_cell\2025101301.xlsx")
        logger.info(response)
        data_list = response.get("data", [])
        order_name = data_list[0].get("orderName", "")
        self.wait_for_transfer_task(filter_text=order_name)
        logger.info("3号站内实验完成")
        return True

    def Bioystation_3_to_2_task(self) -> bool:       
        self.transfer_3_to_2_to_1()
        self.wait_for_transfer_task(filter_text="物料转移任务")
        logger.info("3号站向2号站向1号站转移任务完成")
        return True

    def Bioystation_1_to_2_task(self) -> bool:
        self.transfer_1_to_2()
        self.wait_for_transfer_task(filter_text="物料转移任务")
        logger.info("1号站向2号站转移任务完成")
        logger.info("全流程结束")
        return True
    
    def test_benyao_workstation(self, num1, num2):
        num1 = int(num1)
        num2 = int(num2)
        for i in range(num1):
            print(f"num1 = {num1}")
        for j in range(num2):
            print(f"num1 = {num2}")        

# --------------------------------
if __name__ == "__main__":
    ws = BioyondWorkstation()
    #ws.scheduler_stop()
    #ws.Bioystation_scheduler_start_task()
    ws.scheduler_start()
    # ws.scheduler_start()
    # logger.info("调度启动完成")

    # ws.scheduler_continue()
    # 3.30 上料：读取模板 Excel 自动解析并 POST
    # r1 = ws.auto_feeding4to3_from_xlsx(r"C:\ML\GitHub\Uni-Lab-OS\unilabos\devices\workstation\bioyond_cell\样品导入模板 (8).xlsx")
    # ws.wait_for_transfer_task(filter_text="物料转移任务")
    # logger.info("4号箱向3号箱转运物料转移任务已完成")

    # ws.scheduler_start()
    # print(r1["payload"]["data"])   # 调试模式下可直接看到要发的 JSON items

    # # 新建实验
    # response = ws.create_orders("C:/ML/GitHub/Uni-Lab-OS/unilabos/devices/workstation/bioyond_cell/2025092701.xlsx")
    # logger.info(response)
    # data_list = response.get("data", [])
    # order_name = data_list[0].get("orderName", "")

    # ws.wait_for_transfer_task(filter_text=order_name)
    # ws.wait_for_transfer_task(filter_text='DP20250927001')
    # logger.info("3号站内实验完成")
    # # ws.scheduler_start()
    # # print(res)
    # ws.transfer_3_to_2_to_1()
    # ws.wait_for_transfer_task(filter_text="物料转移任务")
    # logger.info("3号站向2号站向1号站转移任务完成")
        # r321 = self.wait_for_transfer_task()
    #1号站启动
    # ws.transfer_1_to_2()
    #s.wait_for_transfer_task(filter_text="物料转移任务")
    #ogger.info("1号站向2号站转移任务完成")
    #ogger.info("全流程结束")

    # 3.31 下料：同理
    # r2 = ws.auto_batch_outbound_from_xlsx(r"C:/path/样品导入模板 (8).xlsx")
    # print(r2["payload"]["data"])
