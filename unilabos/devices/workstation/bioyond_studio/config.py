# config.py
"""
配置文件 - 包含所有配置信息和映射关系
"""
import os

# ==================== API 基础配置 ====================
# BioyondCellWorkstation 默认配置（包含所有必需参数）
API_CONFIG = {
    # API 连接配置
    "api_host": os.getenv("BIOYOND_API_HOST", "http://172.16.10.169:44388"),
    "api_key": os.getenv("BIOYOND_API_KEY", "8A819E5C"),
    "timeout": int(os.getenv("BIOYOND_TIMEOUT", "30")),
    
    # 报送配置
    "report_token": os.getenv("BIOYOND_REPORT_TOKEN", "CHANGE_ME_TOKEN"),
    
    # HTTP 服务配置
    "HTTP_host": os.getenv("BIOYOND_HTTP_HOST", "172.21.32.91"),  # HTTP服务监听地址，监听计算机飞连ip地址
    "HTTP_port": int(os.getenv("BIOYOND_HTTP_PORT", "8080")),
    "debug_mode": False,# 调试模式
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

    "加样头(大)": ("YB_jia_yang_tou_da", "3a190ca0-b2f6-9aeb-8067-547e72c11469"),
    "加样头(大)板": ("YB_jia_yang_tou_da_1X1_carrier", "a8e714ae-2a4e-4eb9-9614-e4c140ec3f16"),
    # YB信息
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