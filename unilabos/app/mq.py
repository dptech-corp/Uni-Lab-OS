import json
import time
import traceback
from typing import Optional
import uuid

import paho.mqtt.client as mqtt
import ssl
import base64
import hmac
from hashlib import sha1
import tempfile
import os

from unilabos.config.config import MQConfig
from unilabos.app.controler import job_add
from unilabos.app.model import JobAddReq
from unilabos.utils import logger
from unilabos.utils.type_check import TypeEncoder

from paho.mqtt.enums import CallbackAPIVersion


class MQTTClient:
    mqtt_disable = True

    def __init__(self):
        self.mqtt_disable = not MQConfig.lab_id
        self.client_id = f"{MQConfig.group_id}@@@{MQConfig.lab_id}{uuid.uuid4()}"
        logger.info("[MQTT] Client_id: " + self.client_id)
        self.client = mqtt.Client(CallbackAPIVersion.VERSION2, client_id=self.client_id, protocol=mqtt.MQTTv5)
        self._setup_callbacks()

    def _setup_callbacks(self):
        self.client.on_log = self._on_log
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

    def _on_log(self, client, userdata, level, buf):
        # logger.info(f"[MQTT] log: {buf}")
        pass

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        logger.info("[MQTT] Connected with result code " + str(rc))
        client.subscribe(f"labs/{MQConfig.lab_id}/job/start/", 0)
        client.subscribe(f"labs/{MQConfig.lab_id}/pong/", 0)

    def _on_message(self, client, userdata, msg) -> None:
        # logger.info("[MQTT] on_message<<<< " + msg.topic + " " + str(msg.payload))
        try:
            payload_str = msg.payload.decode("utf-8")
            payload_json = json.loads(payload_str)
            if msg.topic == f"labs/{MQConfig.lab_id}/job/start/":
                if "data" not in payload_json:
                    payload_json["data"] = {}
                if "action" in payload_json:
                    payload_json["data"]["action"] = payload_json.pop("action")
                if "action_type" in payload_json:
                    payload_json["data"]["action_type"] = payload_json.pop("action_type")
                if "action_args" in payload_json:
                    payload_json["data"]["action_args"] = payload_json.pop("action_args")
                if "action_kwargs" in payload_json:
                    payload_json["data"]["action_kwargs"] = payload_json.pop("action_kwargs")
                job_req = JobAddReq.model_validate(payload_json)
                data = job_add(job_req)
                return
            elif msg.topic == f"labs/{MQConfig.lab_id}/pong/":
                # 处理pong响应，通知HostNode
                from unilabos.ros.nodes.presets.host_node import HostNode

                host_instance = HostNode.get_instance(0)
                if host_instance:
                    host_instance.handle_pong_response(payload_json)
                return

        except json.JSONDecodeError as e:
            logger.error(f"[MQTT] JSON 解析错误: {e}")
            logger.error(f"[MQTT] Raw message: {msg.payload}")
            logger.error(traceback.format_exc())
        except Exception as e:
            logger.error(f"[MQTT] 处理消息时出错: {e}")
            logger.error(traceback.format_exc())

    def _on_disconnect(self, client, userdata, rc, reasonCode=None, properties=None):
        if rc != 0:
            logger.error(f"[MQTT] Unexpected disconnection {rc}")

    def _setup_ssl_context(self):
        temp_files = []
        try:
            with tempfile.NamedTemporaryFile(mode="w", delete=False) as ca_temp:
                ca_temp.write(MQConfig.ca_content)
                temp_files.append(ca_temp.name)

            with tempfile.NamedTemporaryFile(mode="w", delete=False) as cert_temp:
                cert_temp.write(MQConfig.cert_content)
                temp_files.append(cert_temp.name)

            with tempfile.NamedTemporaryFile(mode="w", delete=False) as key_temp:
                key_temp.write(MQConfig.key_content)
                temp_files.append(key_temp.name)

            context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
            context.load_verify_locations(cafile=temp_files[0])
            context.load_cert_chain(certfile=temp_files[1], keyfile=temp_files[2])
            self.client.tls_set_context(context)
        finally:
            for temp_file in temp_files:
                try:
                    os.unlink(temp_file)
                except Exception as e:
                    pass

    def start(self):
        if self.mqtt_disable:
            logger.warning("MQTT is disabled, skipping connection.")
            return
        userName = f"Signature|{MQConfig.access_key}|{MQConfig.instance_id}"
        password = base64.b64encode(
            hmac.new(MQConfig.secret_key.encode(), self.client_id.encode(), sha1).digest()
        ).decode()

        self.client.username_pw_set(userName, password)
        self._setup_ssl_context()

        # 创建连接线程
        def connect_thread_func():
            try:
                self.client.connect(MQConfig.broker_url, MQConfig.port, 60)
                self.client.loop_start()

                # 添加连接超时检测
                max_attempts = 5
                attempt = 0
                while not self.client.is_connected() and attempt < max_attempts:
                    logger.info(
                        f"[MQTT] 正在连接到 {MQConfig.broker_url}:{MQConfig.port}，尝试 {attempt+1}/{max_attempts}"
                    )
                    time.sleep(3)
                    attempt += 1

                if self.client.is_connected():
                    logger.info(f"[MQTT] 已成功连接到 {MQConfig.broker_url}:{MQConfig.port}")
                else:
                    logger.error(f"[MQTT] 连接超时，可能是账号密码错误或网络问题")
                    self.client.loop_stop()
            except Exception as e:
                logger.error(f"[MQTT] 连接失败: {str(e)}")

        connect_thread_func()
        # connect_thread = threading.Thread(target=connect_thread_func)
        # connect_thread.daemon = True
        # connect_thread.start()

    def stop(self):
        if self.mqtt_disable:
            return
        self.client.disconnect()
        self.client.loop_stop()

    def publish_device_status(self, device_status: dict, device_id, property_name):
        # status = device_status.get(device_id, {})
        if self.mqtt_disable:
            return
        status = {"data": device_status.get(device_id, {}), "device_id": device_id, "timestamp": time.time()}
        address = f"labs/{MQConfig.lab_id}/devices/"
        self.client.publish(address, json.dumps(status), qos=2)
        logger.info(f"Device {device_id} status published: address: {address}, {status}")

    def publish_job_status(self, feedback_data: dict, job_id: str, status: str, return_info: Optional[str] = None):
        if self.mqtt_disable:
            return
        if return_info is None:
            return_info = "{}"
        jobdata = {"job_id": job_id, "data": feedback_data, "status": status, "return_info": return_info}
        self.client.publish(f"labs/{MQConfig.lab_id}/job/list/", json.dumps(jobdata), qos=2)

    def publish_registry(self, device_id: str, device_info: dict, print_debug: bool = True):
        if self.mqtt_disable:
            return
        address = f"labs/{MQConfig.lab_id}/registry/"
        registry_data = json.dumps({device_id: device_info}, ensure_ascii=False, cls=TypeEncoder)
        self.client.publish(address, registry_data, qos=2)
        if print_debug:
            logger.debug(f"Registry data published: address: {address}, {registry_data}")

    def publish_actions(self, action_id: str, action_info: dict):
        if self.mqtt_disable:
            return
        address = f"labs/{MQConfig.lab_id}/actions/"
        self.client.publish(address, json.dumps(action_info), qos=2)
        logger.debug(f"Action data published: address: {address}, {action_id}, {action_info}")

    def send_ping(self, ping_id: str, timestamp: float):
        """发送ping消息到服务端"""
        if self.mqtt_disable:
            return
        address = f"labs/{MQConfig.lab_id}/ping/"
        ping_data = {"ping_id": ping_id, "client_timestamp": timestamp, "type": "ping"}
        self.client.publish(address, json.dumps(ping_data), qos=2)

    def setup_pong_subscription(self):
        """设置pong消息订阅"""
        if self.mqtt_disable:
            return
        pong_topic = f"labs/{MQConfig.lab_id}/pong/"
        self.client.subscribe(pong_topic, 0)
        logger.debug(f"Subscribed to pong topic: {pong_topic}")

    def handle_pong(self, pong_data: dict):
        """处理pong响应（这个方法会在收到pong消息时被调用）"""
        logger.debug(f"Pong received: {pong_data}")
        # 这里会被HostNode的ping-pong处理逻辑调用
        pass


mqtt_client = MQTTClient()

if __name__ == "__main__":
    mqtt_client.start()
