import requests
import json
from datetime import datetime
def test_benyao_api():
    # 配置信息
    ip_addr = "192.168.1.200"
    port = 44386
    #url = f"http://{ip_addr}:{port}/api/lims/scheduler/scheduler-status"
    #url = f"http://{ip_addr}:{port}/api/lims/order/order-list-status"
    url = f"http://{ip_addr}:{port}/api/lims/storage/stock-material"
    apiKey = "8A819E5C"  # 请替换为实际apiKey
    
    # 构造请求体
    request_data = {
        "apiKey": apiKey,
        "requestTime": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%fZ"),  # 示例：2025-08-15T10:00:00.000Z
        "data": {
                "typeMode": 1,
                "includeDetail": True
            }
    
    }
    
    #request_data = {
    #    "apiKey": apiKey,
    #    "requestTime": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%fZ"),  # 示例：2025-08-15T10:00:00.000Z
    #    "data": 
    #}
    
    
    print(request_data)
    # 发送POST请求
    try:
        response = requests.post(url, json=request_data, timeout=10)
        response.raise_for_status()  # 检查HTTP状态码
    
        # 解析响应
        result = response.json()
        print("响应状态码:", response.status_code)
        print("响应内容:")
        print(json.dumps(result, indent=2, ensure_ascii=False))
    
    except requests.exceptions.RequestException as e:
        print("请求失败:", e)
    except json.JSONDecodeError as e:
        print("JSON解析失败:", e)
    
if __name__ == "__main__":
    test_benyao_api()