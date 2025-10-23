# config.py
"""
配置文件 - 包含所有配置信息和映射关系
"""
import os

# ==================== API 基础配置 ====================
# 支持通过环境变量覆盖默认值
API_CONFIG = {
    "api_key": os.getenv("BIOYOND_API_KEY", "8A819E5C"),
    "api_host": os.getenv("BIOYOND_API_HOST", "http://172.16.11.219:44388"),
}

# ==================== 完整的 Bioyond 配置 ====================
# BioyondCellWorkstation 默认配置（包含所有必需参数）
BIOYOND_FULL_CONFIG = {
    # API 连接配置
    "base_url": os.getenv("BIOYOND_API_HOST", "http://172.16.11.219:44388"),
    "api_key": os.getenv("BIOYOND_API_KEY", "8A819E5C"),
    "timeout": int(os.getenv("BIOYOND_TIMEOUT", "30")),
    
    # 报送配置
    "report_token": os.getenv("BIOYOND_REPORT_TOKEN", "CHANGE_ME_TOKEN"),
    
    # HTTP 服务配置
    "HTTP_host": os.getenv("BIOYOND_HTTP_HOST", "0.0.0.0"),  # HTTP服务监听地址（0.0.0.0 表示监听所有网络接口）
    "HTTP_port": int(os.getenv("BIOYOND_HTTP_PORT", "8080")),
    "report_ip": os.getenv("BIOYOND_REPORT_IP", "172.21.32.172"),  # 报送给 Bioyond 的本机IP地址（留空则自动检测）
    # 调试模式
    "debug_mode": os.getenv("BIOYOND_DEBUG_MODE", "False").lower() == "true",
}

# 工作流映射配置
WORKFLOW_MAPPINGS = {
    "reactor_taken_out": "",
    "reactor_taken_in": "",
    "Solid_feeding_vials": "",
    "Liquid_feeding_vials(non-titration)": "",
    "Liquid_feeding_solvents": "",
    "Liquid_feeding(titration)": "",
    "liquid_feeding_beaker": "",
    "Drip_back": "",
}

# 工作流名称到DisplaySectionName的映射
WORKFLOW_TO_SECTION_MAP = {
    'reactor_taken_in': '反应器放入',
    'liquid_feeding_beaker': '液体投料-烧杯',
    'Liquid_feeding_vials(non-titration)': '液体投料-小瓶（非滴定）',
    'Liquid_feeding_solvents': '液体投料-溶剂',
    'Solid_feeding_vials': '固体投料-小瓶',
    'Liquid_feeding(titration)': '液体投料-滴定',
    'reactor_taken_out': '反应器取出'
}

# 库位映射配置
WAREHOUSE_MAPPING = {
    "粉末堆栈": {
        "uuid": "",
        "site_uuids": {
            # 样品板
            "A1": "3a14198e-6929-31f0-8a22-0f98f72260df",
            "A2": "3a14198e-6929-4379-affa-9a2935c17f99",
            "A3": "3a14198e-6929-56da-9a1c-7f5fbd4ae8af",
            "A4": "3a14198e-6929-5e99-2b79-80720f7cfb54",
            "B1": "3a14198e-6929-f525-9a1b-1857552b28ee",
            "B2": "3a14198e-6929-bf98-0fd5-26e1d68bf62d",
            "B3": "3a14198e-6929-2d86-a468-602175a2b5aa",
            "B4": "3a14198e-6929-1a98-ae57-e97660c489ad",
            # 分装板
            "C1": "3a14198e-6929-46fe-841e-03dd753f1e4a",
            "C2": "3a14198e-6929-1bc9-a9bd-3b7ca66e7f95",
            "C3": "3a14198e-6929-72ac-32ce-9b50245682b8",
            "C4": "3a14198e-6929-3bd8-e6c7-4a9fd93be118",
            "D1": "3a14198e-6929-8a0b-b686-6f4a2955c4e2",
            "D2": "3a14198e-6929-dde1-fc78-34a84b71afdf",
            "D3": "3a14198e-6929-a0ec-5f15-c0f9f339f963",
            "D4": "3a14198e-6929-7ac8-915a-fea51cb2e884"
        }
    },
    "溶液堆栈": {
        "uuid": "",
        "site_uuids": {
            "A1": "3a14198e-d724-e036-afdc-2ae39a7f3383",
            "A2": "3a14198e-d724-afa4-fc82-0ac8a9016791",
            "A3": "3a14198e-d724-ca48-bb9e-7e85751e55b6",
            "A4": "3a14198e-d724-df6d-5e32-5483b3cab583",
            "B1": "3a14198e-d724-d818-6d4f-5725191a24b5",
            "B2": "3a14198e-d724-be8a-5e0b-012675e195c6",
            "B3": "3a14198e-d724-cc1e-5c2c-228a130f40a8",
            "B4": "3a14198e-d724-1e28-c885-574c3df468d0",
            "C1": "3a14198e-d724-b5bb-adf3-4c5a0da6fb31",
            "C2": "3a14198e-d724-ab4e-48cb-817c3c146707",
            "C3": "3a14198e-d724-7f18-1853-39d0c62e1d33",
            "C4": "3a14198e-d724-28a2-a760-baa896f46b66",
            "D1": "3a14198e-d724-d378-d266-2508a224a19f",
            "D2": "3a14198e-d724-f56e-468b-0110a8feb36a",
            "D3": "3a14198e-d724-0cf1-dea9-a1f40fe7e13c",
            "D4": "3a14198e-d724-0ddd-9654-f9352a421de9"
        }
    },
    "试剂堆栈": {
        "uuid": "",
        "site_uuids": {
            "A1": "3a14198c-c2cf-8b40-af28-b467808f1c36",
            "A2": "3a14198c-c2d0-f3e7-871a-e470d144296f",
            "A3": "3a14198c-c2d0-dc7d-b8d0-e1d88cee3094",
            "A4": "3a14198c-c2d0-2070-efc8-44e245f10c6f",
            "B1": "3a14198c-c2d0-354f-39ad-642e1a72fcb8",
            "B2": "3a14198c-c2d0-1559-105d-0ea30682cab4",
            "B3": "3a14198c-c2d0-725e-523d-34c037ac2440",
            "B4": "3a14198c-c2d0-efce-0939-69ca5a7dfd39"
        }
    },
    "粉末加样头堆栈": {
        "uuid": "",
        "site_uuids": {
            "A01": "3a19da56-1379-20c8-5886-f7c4fbcb5733",
            "B01": "3a19da56-1379-2424-d751-fe6e94cef938",
            "C01": "3a19da56-1379-271c-03e3-6bdb590e395e",
            "D01": "3a19da56-1379-277f-2b1b-0d11f7cf92c6",
            "E01": "3a19da56-1379-2f1c-a15b-e01db90eb39a",
            "F01": "3a19da56-1379-3fa1-846b-088158ac0b3d",
            "G01": "3a19da56-1379-5aeb-d0cd-d3b4609d66e1",
            "H01": "3a19da56-1379-6077-8258-bdc036870b78",
            "I01": "3a19da56-1379-863b-a120-f606baf04617",
            "J01": "3a19da56-1379-8a74-74e5-35a9b41d4fd5",
            "K01": "3a19da56-1379-b270-b7af-f18773918abe",
            "L01": "3a19da56-1379-ba54-6d78-fd770a671ffc",
            "M01": "3a19da56-1379-c22d-c96f-0ceb5eb54a04",
            "N01": "3a19da56-1379-d64e-c6c5-c72ea4829888",
            "O01": "3a19da56-1379-d887-1a3c-6f9cce90f90e",
            "P01": "3a19da56-1379-e77d-0e65-7463b238a3b9",
            "Q01": "3a19da56-1379-edf6-1472-802ddb628774",
            "R01": "3a19da56-1379-f281-0273-e0ef78f0fd97",
            "S01": "3a19da56-1379-f924-7f68-df1fa51489f4",
            "T01": "3a19da56-1379-ff7c-1745-07e200b44ce2"
        }
    }
}

# 物料类型配置
MATERIAL_TYPE_MAPPINGS = {
    "烧杯": ("BIOYOND_PolymerStation_1FlaskCarrier", "3a14196b-24f2-ca49-9081-0cab8021bf1a"),
    "试剂瓶": ("BIOYOND_PolymerStation_1BottleCarrier", ""),
    "样品板": ("BIOYOND_PolymerStation_6StockCarrier", "3a14196e-b7a0-a5da-1931-35f3000281e9"),
    "分装板": ("BIOYOND_PolymerStation_6VialCarrier", "3a14196e-5dfe-6e21-0c79-fe2036d052c4"),
    "样品瓶": ("BIOYOND_PolymerStation_Solid_Stock", "3a14196a-cf7d-8aea-48d8-b9662c7dba94"),
    "90%分装小瓶": ("BIOYOND_PolymerStation_Solid_Vial", "3a14196c-cdcf-088d-dc7d-5cf38f0ad9ea"),
    "10%分装小瓶": ("BIOYOND_PolymerStation_Liquid_Vial", "3a14196c-76be-2279-4e22-7310d69aed68"),
    "20ml分液瓶": ("BIOYOND_PolymerStation_6x20ml_DispensingVialCarrier", "3a14196e-5dfe-6e21-0c79-fe2036d052c4"),
    "100ml液体": ("BIOYOND_PolymerStation_100ml_Liquid_Bottle", "d37166b3-ecaa-481e-bd84-3032b795ba07"),
    "液": ("BIOYOND_PolymerStation_Liquid_Bottle", "3a190ca1-2add-2b23-f8e1-bbd348b7f790"),
    "高粘液": ("BIOYOND_PolymerStation_High_Viscosity_Liquid_Bottle", "abe8df30-563d-43d2-85e0-cabec59ddc16"),
    "加样头(大)": ("BIOYOND_PolymerStation_Large_Dispense_Head", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    "5ml分液瓶板": ("BIOYOND_PolymerStation_6x5ml_DispensingVialCarrier", "3a192fa4-007d-ec7b-456e-2a8be7a13f23"),
    "5ml分液瓶": ("BIOYOND_PolymerStation_5ml_Dispensing_Vial", "3a192c2a-ebb7-58a1-480d-8b3863bf74f4"),
    "20ml分液瓶板": ("BIOYOND_PolymerStation_6x20ml_DispensingVialCarrier", "3a192fa4-47db-3449-162a-eaf8aba57e27"),
    "配液瓶(小)板": ("BIOYOND_PolymerStation_6x_SmallSolutionBottleCarrier", "3a190c8b-3284-af78-d29f-9a69463ad047"),
    "配液瓶(小)": ("BIOYOND_PolymerStation_Small_Solution_Bottle", "3a190c8c-fe8f-bf48-0dc3-97afc7f508eb"),
    "配液瓶(大)板": ("BIOYOND_PolymerStation_4x_LargeSolutionBottleCarrier", "53e50377-32dc-4781-b3c0-5ce45bc7dc27"),
    "配液瓶(大)": ("BIOYOND_PolymerStation_Large_Solution_Bottle", "19c52ad1-51c5-494f-8854-576f4ca9c6ca"),
    "加样头(大)板": ("BIOYOND_PolymerStation_6x_LargeDispenseHeadCarrier", "a8e714ae-2a4e-4eb9-9614-e4c140ec3f16"),
    "适配器块": ("BIOYOND_PolymerStation_AdapterBlock", "efc3bb32-d504-4890-91c0-b64ed3ac80cf"),
    "枪头盒": ("BIOYOND_PolymerStation_TipBox", "3a192c2e-20f3-a44a-0334-c8301839d0b3"),
    "枪头": ("BIOYOND_PolymerStation_Pipette_Tip", "b6196971-1050-46da-9927-333e8dea062d"),
}

# 步骤参数配置（各工作流的步骤UUID）
WORKFLOW_STEP_IDS = {
    "reactor_taken_in": {
        "config": ""
    },
    "liquid_feeding_beaker": {
        "liquid": "",
        "observe": ""
    },
    "liquid_feeding_vials_non_titration": {
        "liquid": "",
        "observe": ""
    },
    "liquid_feeding_solvents": {
        "liquid": "",
        "observe": ""
    },
    "solid_feeding_vials": {
        "feeding": "",
        "observe": ""
    },
    "liquid_feeding_titration": {
        "liquid": "",
        "observe": ""
    },
    "drip_back": {
        "liquid": "",
        "observe": ""
    }
}

LOCATION_MAPPING = {}