# MQTT配置
class MQConfig:
    lab_id = ""
    instance_id = ""
    access_key = ""
    secret_key = ""
    group_id = ""
    broker_url = ""
    port = 1883

    ca_file = "CA.crt"
    cert_file = "lab.crt"
    key_file = "lab.key"

# HTTP配置
class HTTPConfig:
    remote_addr = "https://uni-lab.bohrium.com/api/v1"
