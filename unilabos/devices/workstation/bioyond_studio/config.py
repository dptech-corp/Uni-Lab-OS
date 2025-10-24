# config.py
"""
配置文件 - 包含所有配置信息和映射关系
"""
import os

# ==================== API 基础配置 ====================


# ==================== 完整的 Bioyond 配置 ====================
# BioyondCellWorkstation 默认配置（包含所有必需参数）
API_CONFIG = {
    # API 连接配置
    "api_host": os.getenv("BIOYOND_API_HOST", "http://172.16.11.219:44388"),
    "api_key": os.getenv("BIOYOND_API_KEY", "8A819E5C"),
    "timeout": int(os.getenv("BIOYOND_TIMEOUT", "30")),
    
    # 报送配置
    "report_token": os.getenv("BIOYOND_REPORT_TOKEN", "CHANGE_ME_TOKEN"),
    
    # HTTP 服务配置
    "HTTP_host": os.getenv("BIOYOND_HTTP_HOST", "0.0.0.0"),  # HTTP服务监听地址（0.0.0.0 表示监听所有网络接口）
    "HTTP_port": int(os.getenv("BIOYOND_HTTP_PORT", "8080")),
    "report_ip": os.getenv("BIOYOND_REPORT_IP", "172.21.32.172"),  # 报送给 Bioyond 的本机IP地址（留空则自动检测）
    # 调试模式
    "debug_mode": False,
}

# 库位映射配置
WAREHOUSE_MAPPING = {
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

SOLID_LIQUID_MAPPINGS = {
    # 固体
    "LiDFOB": {
        "typeId": "3a190ca0-b2f6-9aeb-8067-547e72c11469",
        "code": "",
        "barCode": "",
        "name": "LiDFOB",
        "unit": "g",
        "parameters": "",
        "quantity": "2",
        "warningQuantity": "1",
        "details": []
    },
    # "LiPF6": {
    #     "typeId": "3a190ca0-b2f6-9aeb-8067-547e72c11469",
    #     "code": "",
    #     "barCode": "",
    #     "name": "LiPF6",
    #     "unit": "g",
    #     "parameters": "",
    #     "quantity": 2,
    #     "warningQuantity": 1,
    #     "details": []
    # },
    # "LiFSI": {
    #     "typeId": "3a190ca0-b2f6-9aeb-8067-547e72c11469",
    #     "code": "",
    #     "barCode": "",
    #     "name": "LiFSI",
    #     "unit": "g",
    #     "parameters": {"Density": "1.533"},
    #     "quantity": 2,
    #     "warningQuantity": 1,
    #     "details": [{}]
    # },
    # "DTC": {
    #     "typeId": "3a190ca0-b2f6-9aeb-8067-547e72c11469",
    #     "code": "",
    #     "barCode": "",
    #     "name": "DTC",
    #     "unit": "g",
    #     "parameters": {"Density": "1.533"},
    #     "quantity": 2,
    #     "warningQuantity": 1,
    #     "details": [{}]
    # },
    # "LiPO2F2": {
    #     "typeId": "3a190ca0-b2f6-9aeb-8067-547e72c11469",
    #     "code": "",
    #     "barCode": "",
    #     "name": "LiPO2F2",
    #     "unit": "g",
    #     "parameters": {"Density": "1.533"},
    #     "quantity": 2,
    #     "warningQuantity": 1,
    #     "details": [{}]
    # },
    # 液体
    # "SA": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "EC": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "VC": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "AND": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "HTCN": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "DENE": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "TMSP": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "TMSB": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "EP": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "DEC": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "EMC": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "SN": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "DMC": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    # "FEC": ("BIOYOND_PolymerStation_Solid_Stock", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
}

WORKFLOW_MAPPINGS = {}

LOCATION_MAPPING = {}