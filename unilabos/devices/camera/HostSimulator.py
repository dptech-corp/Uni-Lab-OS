#!/usr/bin/env python3
import asyncio
import json
import signal
import subprocess
import sys

import requests  # type: ignore
import websockets

# BACKEND = "ws://192.168.31.31:48197/"
BACKEND = "wss://sciol.ac.cn/"

class HostSimulator:
    def __init__(
        self,
        host_id="demo-host",
        backend_url=f"{BACKEND}api/realtime/signal/host",
    ):
        """host_id matches the front-end CameraMonitor hostId.
            backend_url uses service port 48197 + /api/realtime/signal/host.
        Local test URL:
        ws://localhost:48197/api/realtime/signal/host
        """
        self.host_id = host_id
        self.backend_url = f"{backend_url}/{host_id}"
        # SRS 公网地址 (通过 FRP + K8s 部署)
        # RTMP 推流: 使用公网地址 + 非标准端口 4499
        self.srs_rtmp = "rtmp://srs.sciol.ac.cn:4499/live/camera-01"
        # WebRTC API: 使用 HTTPS (通过 NGINX Gateway)
        self.srs_api = "https://srs.sciol.ac.cn/rtc/v1/play/"
        self.ffmpeg_process = None
        self.ws = None

    async def connect_to_backend(self):
        """连接到后端 WebSocket 服务器"""
        print(f"正在连接到后端: {self.backend_url}")
        self.ws = await websockets.connect(self.backend_url)  # type: ignore
        print(f"✓ Host {self.host_id} 已连接到后端")

        # 连接成功后自动启动推流,确保 GOP 缓存可用
        print("自动启动摄像头推流...")
        self.start_ffmpeg_stream()
        print("推流已就绪,等待 WebRTC 连接...")
        print()

    async def handle_commands(self):
        """处理来自后端的命令"""
        try:
            async for message in self.ws:  # type: ignore
                data = json.loads(message)
                print(f"收到命令: {data}")

                if data.get("command") == "start_stream":
                    camera_id = data.get("cameraId")
                    print(f"启动摄像头推流: {camera_id}")
                    self.start_ffmpeg_stream()

                elif data.get("command") == "stop_stream":
                    print("停止推流")
                    self.stop_ffmpeg_stream()

                elif data.get("type") == "offer":
                    # 处理 WebRTC Offer
                    print("收到 WebRTC Offer，正在通过 SRS 生成 Answer...")
                    answer_sdp = self.handle_webrtc_offer(data.get("sdp"))

                    if answer_sdp:
                        # 发送 Answer 回前端
                        response = {
                            "type": "answer",
                            "sdp": answer_sdp,
                            "cameraId": data.get("cameraId"),
                            "hostId": self.host_id,
                        }
                        # send answer back
                        payload = json.dumps(response)
                        await self.ws.send(payload)  # type: ignore
                        print("✓ Answer SDP 已发送")

        except websockets.exceptions.ConnectionClosed:
            print("WebSocket 连接已关闭")
        except Exception as e:
            print(f"错误: {e}")

    def start_ffmpeg_stream(self):
        """使用 FFmpeg 推流摄像头到 SRS"""
        if self.ffmpeg_process:
            print("推流已在运行")
            return

        import platform

        system = platform.system()

        if system == "Darwin":  # macOS
            # Mac 摄像头设备 - 使用 avfoundation 驱动
            cmd = [
                "ffmpeg",
                "-f",
                "avfoundation",
                "-framerate",
                "30",
                "-video_size",
                "1280x720",
                "-i",
                "0",  # 0 是默认摄像头
                # 编码参数 - 低延迟优化
                "-c:v",
                "libx264",
                "-preset",
                "ultrafast",
                "-tune",
                "zerolatency",
                "-profile:v",
                "baseline",
                "-b:v",
                "2M",
                "-maxrate",
                "2M",
                "-bufsize",
                "4M",
                "-g",
                "60",
                "-keyint_min",
                "30",
                "-sc_threshold",
                "0",
                "-pix_fmt",
                "yuv420p",
                # 输出到 RTMP
                "-f",
                "flv",
                self.srs_rtmp,
            ]
        elif system == "Linux":  # Linux
            # Linux 摄像头设备 - 使用 Video4Linux (v4l2)
            cmd = [
                "ffmpeg",
                "-f",
                "v4l2",  # Linux 视频输入
                "-framerate",
                "30",
                "-video_size",
                "1280x720",
                "-i",
                "/dev/video0",  # 第一个摄像头
                # 编码参数 - 低延迟优化 + 频繁关键帧
                "-c:v",
                "libx264",
                "-preset",
                "ultrafast",
                "-tune",
                "zerolatency",
                "-profile:v",
                "baseline",
                "-b:v",
                "1M",  # 降低码率适应 10fps
                "-maxrate",
                "1M",
                "-bufsize",
                "2M",
                "-g",
                "10",  # 每 10 帧一个关键帧 (10fps 下约 1 秒)
                "-keyint_min",
                "10",
                "-sc_threshold",
                "0",
                "-pix_fmt",
                "yuv420p",
                "-x264-params",
                "bframes=0",  # 禁用 B 帧,进一步降低延迟
                # 输出到 RTMP
                "-f",
                "flv",
                self.srs_rtmp,
            ]
        else:
            print(f"✗ 不支持的操作系统: {system}")
            return

        print(f"启动 FFmpeg: {' '.join(cmd)}")

        try:
            # 使用 PIPE 捕获输出，但设置非阻塞以便实时查看错误
            self.ffmpeg_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,  # 合并到 stdout 便于查看
                universal_newlines=True,
                bufsize=1,
            )
            print("✓ FFmpeg 推流已启动")
            print(f"  系统: {system}")
            print(f"  PID: {self.ffmpeg_process.pid}")
            print(f"  推流地址: {self.srs_rtmp}")

            # 启动线程监听 FFmpeg 输出
            import threading

            def monitor_ffmpeg():
                if self.ffmpeg_process and self.ffmpeg_process.stdout:
                    for line in self.ffmpeg_process.stdout:
                        if line.strip():
                            print(f"  [FFmpeg] {line.strip()}")

            threading.Thread(target=monitor_ffmpeg, daemon=True).start()

        except Exception as e:
            print(f"✗ FFmpeg 启动失败: {e}")
            if system == "Darwin":
                print("提示: 请确保已安装 FFmpeg (brew install ffmpeg)")
            elif system == "Linux":
                print("提示: 请确保已安装 FFmpeg (apt install ffmpeg)")
                print("     并检查摄像头设备: ls -l /dev/video*")

    def stop_ffmpeg_stream(self):
        """停止 FFmpeg 推流"""
        if self.ffmpeg_process:
            self.ffmpeg_process.send_signal(signal.SIGINT)
            self.ffmpeg_process.wait(timeout=5)
            self.ffmpeg_process = None
            print("✓ FFmpeg 推流已停止")

    def handle_webrtc_offer(self, offer_sdp):
        """
        处理 WebRTC Offer，通过 SRS API 生成 Answer
        注意：前端是播放器（recvonly），所以使用 SRS 的 play API
        """
        try:
            # 使用公网 WebRTC 地址 (端口 4500)
            payload = {
                "api": self.srs_api,
                "streamurl": "webrtc://srs.sciol.ac.cn:4500/live/camera-01",
                "sdp": offer_sdp,
            }

            response = requests.post(
                self.srs_api,
                json=payload,
                headers={"Content-Type": "application/json"},
                timeout=5,
            )

            if response.status_code == 200:
                data = response.json()
                return data.get("sdp")
            else:
                print(f"SRS API 错误: {response.status_code} - {response.text}")
                return None

        except Exception as e:
            print(f"处理 WebRTC Offer 失败: {e}")
            return None

    def cleanup(self):
        """清理资源"""
        print("\n正在清理...")
        self.stop_ffmpeg_stream()

    async def run(self):
        """运行主循环"""
        try:
            await self.connect_to_backend()
            await self.handle_commands()
        except KeyboardInterrupt:
            print("\n收到中断信号")
        except Exception as e:
            print(f"错误: {e}")
        finally:
            self.cleanup()


def main():
    print("========================================")
    print("WebRTC Host 模拟器")
    print("========================================")
    print()

    # 检查依赖
    import shutil

    if not shutil.which("ffmpeg"):
        print("✗ 错误: 未找到 FFmpeg")
        print("请安装: brew install ffmpeg")
        sys.exit(1)

    print("✓ FFmpeg 已安装")
    print()

    # 创建并运行模拟器
    # Use same host_id as front-end (CameraMonitor hostId prop)
    simulator = HostSimulator(
        host_id="demo-host",
        backend_url=f"{BACKEND}api/realtime/signal/host",
    )

    # 设置信号处理
    def signal_handler(sig, frame):
        simulator.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 运行
    asyncio.run(simulator.run())


if __name__ == "__main__":
    main()