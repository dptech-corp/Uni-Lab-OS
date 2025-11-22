#!/usr/bin/env python3
import asyncio
import json
import subprocess
import sys
import threading
from typing import Optional, Dict, Any

import requests
import websockets


class CameraController:
    """
    Uni-Lab-OS 摄像头驱动（driver 形式）
    启动 Uni-Lab-OS 后，立即开始推流

    - WebSocket 信令：通过 signal_backend_url 连接到后端
      例如: wss://sciol.ac.cn/api/realtime/signal/host/<host_id>
    - 媒体服务器：通过 rtmp_url / webrtc_api / webrtc_stream_url
      当前配置为 SRS，与独立 HostSimulator 独立运行脚本保持一致。
    """

    def __init__(
        self,
        host_id: str = "demo-host",

        # （1）信令后端（WebSocket）
        signal_backend_url: str = "wss://sciol.ac.cn/api/realtime/signal/host",

        # （2）媒体后端（RTMP + WebRTC API）
        rtmp_url: str = "rtmp://srs.sciol.ac.cn:4499/live/camera-01",
        webrtc_api: str = "https://srs.sciol.ac.cn/rtc/v1/play/",
        webrtc_stream_url: str = "webrtc://srs.sciol.ac.cn:4500/live/camera-01",
    ):
        self.host_id = host_id

        # 拼接最终的 WebSocket URL：.../host/<host_id>
        signal_backend_url = signal_backend_url.rstrip("/")
        if not signal_backend_url.endswith("/host"):
            signal_backend_url = signal_backend_url + "/host"
        self.signal_backend_url = f"{signal_backend_url}/{host_id}"

        # 媒体服务器配置（与 HostSimulator 保持一致）
        self.rtmp_url = rtmp_url
        self.webrtc_api = webrtc_api
        self.webrtc_stream_url = webrtc_stream_url

        # 运行时状态
        self._ws: Optional[object] = None
        self._ffmpeg_process: Optional[subprocess.Popen] = None
        self._running = False
        self._loop_task: Optional[asyncio.Future] = None

        # 事件循环 & 线程
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._loop_thread: Optional[threading.Thread] = None

        # 这里不传 config，使用构造参数作为默认配置
        try:
            self.start()
        except Exception as e:
            # 日志调整: 构造阶段只打印错误
            print(f"[CameraController] __init__ auto start failed: {e}", file=sys.stderr)

    # ---------------------------------------------------------------------
    # 对外暴露的方法：供 Uni-Lab-OS 调用
    # ---------------------------------------------------------------------

    def start(self, config: Optional[Dict[str, Any]] = None):
        """
        启动 Camera 连接 & 消息循环，并在启动时就开启 FFmpeg 推流，
        """

        if self._running:
            return {"status": "already_running", "host_id": self.host_id}

        # 应用 config 覆盖（如果有）
        if config:
            cfg_host_id = config.get("host_id")
            if cfg_host_id:
                self.host_id = cfg_host_id

            signal_backend_url = config.get("signal_backend_url")
            if signal_backend_url:
                signal_backend_url = signal_backend_url.rstrip("/")
                if not signal_backend_url.endswith("/host"):
                    signal_backend_url = signal_backend_url + "/host"
                self.signal_backend_url = f"{signal_backend_url}/{self.host_id}"

            self.rtmp_url = config.get("rtmp_url", self.rtmp_url)
            self.webrtc_api = config.get("webrtc_api", self.webrtc_api)
            self.webrtc_stream_url = config.get(
                "webrtc_stream_url", self.webrtc_stream_url
            )

        self._running = True

        # === start 时启动 FFmpeg 推流 ===
        # 日志调整: 不再输出正常启动信息，保留异常打印
        self._start_ffmpeg()
        # =================================================

        # 创建新的事件循环和线程（用于 WebSocket 信令）
        self._loop = asyncio.new_event_loop()

        def loop_runner(loop: asyncio.AbstractEventLoop):
            asyncio.set_event_loop(loop)
            try:
                loop.run_forever()
            except Exception as e:
                # 日志调整: 只打印循环异常
                print(f"[CameraController] event loop error: {e}", file=sys.stderr)

        self._loop_thread = threading.Thread(
            target=loop_runner, args=(self._loop,), daemon=True
        )
        self._loop_thread.start()

        # 在这个新 loop 上调度主协程
        self._loop_task = asyncio.run_coroutine_threadsafe(
            self._run_main_loop(), self._loop
        )

        return {
            "status": "started",
            "host_id": self.host_id,
            "signal_backend_url": self.signal_backend_url,
            "rtmp_url": self.rtmp_url,
            "webrtc_api": self.webrtc_api,
            "webrtc_stream_url": self.webrtc_stream_url,
        }

    def stop(self) -> Dict[str, Any]:
        """
        停止推流 & 断开 WebSocket，并关闭事件循环线程。
        """
        self._running = False

        # 停止 ffmpeg
        self._stop_ffmpeg()

        # 在事件循环线程中关闭 WebSocket
        if self._ws and self._loop is not None:

            async def close_ws():
                try:
                    await self._ws.close()
                except Exception as e:
                    print(
                        f"[CameraController] error when closing WebSocket: {e}",
                        file=sys.stderr,
                    )

            asyncio.run_coroutine_threadsafe(close_ws(), self._loop)

        # 停止事件循环
        if self._loop is not None:
            try:
                self._loop.call_soon_threadsafe(self._loop.stop)
            except Exception as e:
                print(
                    f"[CameraController] error when stopping event loop: {e}",
                    file=sys.stderr,
                )

        # 等待线程结束
        if self._loop_thread is not None:
            try:
                self._loop_thread.join(timeout=5)
            except Exception as e:
                print(
                    f"[CameraController] error when joining loop thread: {e}",
                    file=sys.stderr,
                )

        # 清理状态
        self._ws = None
        self._loop_task = None
        self._loop = None
        self._loop_thread = None

        return {"status": "stopped", "host_id": self.host_id}

    def get_status(self) -> Dict[str, Any]:
        """
        查询当前状态，方便在 Uni-Lab-OS 中做监控。
        """
        ws_closed = None
        if self._ws is not None:
            ws_closed = getattr(self._ws, "closed", None)

        if ws_closed is None:
            websocket_connected = self._ws is not None
        else:
            websocket_connected = (self._ws is not None) and (not ws_closed)

        return {
            "host_id": self.host_id,
            "running": self._running,
            "websocket_connected": websocket_connected,
            "ffmpeg_running": bool(
                self._ffmpeg_process and self._ffmpeg_process.poll() is None
            ),
            "signal_backend_url": self.signal_backend_url,
            "rtmp_url": self.rtmp_url,
        }

    # ---------------------------------------------------------------------
    # 内部实现逻辑：WebSocket 循环 / FFmpeg / WebRTC Offer 处理
    # ---------------------------------------------------------------------

    async def _run_main_loop(self):
        """
        后台主循环：
        - 建立 WebSocket 连接
        - 不断接收后端的命令/offer
        """
        # 日志调整: 移除正常“loop started/exited”打印
        while self._running:
            try:
                async with websockets.connect(self.signal_backend_url) as ws:
                    self._ws = ws
                    await self._recv_loop()
            except Exception as e:
                if self._running:
                    print(
                        f"[CameraController] WebSocket connection error: {e}",
                        file=sys.stderr,
                    )
                    await asyncio.sleep(3)

    async def _recv_loop(self):
        """
        WebSocket 收消息循环
        """
        assert self._ws is not None
        ws = self._ws

        async for message in ws:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                # 日志调整: 仅在格式错误时打印
                print(
                    f"[CameraController] received non-JSON message: {message}",
                    file=sys.stderr,
                )
                continue

            try:
                await self._handle_message(data)
            except Exception as e:
                print(
                    f"[CameraController] error while handling message {data}: {e}",
                    file=sys.stderr,
                )

    async def _handle_message(self, data: Dict[str, Any]):
        """
        处理来自信令后端的消息：
        - command: start_stream / stop_stream
        - type: offer (WebRTC)
        """
        cmd = data.get("command")
        if cmd == "start_stream":
            # 日志调整: 不打印“收到 start_stream”，只在失败时打印
            try:
                self._start_ffmpeg()
            except Exception as e:
                print(
                    f"[CameraController] error when starting FFmpeg on start_stream: {e}",
                    file=sys.stderr,
                )
            return

        if cmd == "stop_stream":
            # 日志调整: 不打印正常 stop
            try:
                self._stop_ffmpeg()
            except Exception as e:
                print(
                    f"[CameraController] error when stopping FFmpeg on stop_stream: {e}",
                    file=sys.stderr,
                )
            return

        if data.get("type") == "offer":
            offer_sdp = data.get("sdp", "")
            camera_id = data.get("cameraId", "camera-01")
            try:
                answer_sdp = self._handle_webrtc_offer(offer_sdp)
            except Exception as e:
                print(
                    f"[CameraController] error when handling WebRTC offer: {e}",
                    file=sys.stderr,
                )
                return

            # 把 answer 发回信令后端
            if self._ws:
                answer_payload = {
                    "type": "answer",
                    "sdp": answer_sdp,
                    "cameraId": camera_id,
                    "hostId": self.host_id,
                }
                try:
                    await self._ws.send(json.dumps(answer_payload))
                except Exception as e:
                    print(
                        f"[CameraController] error when sending WebRTC answer: {e}",
                        file=sys.stderr,
                    )

    # ------------------------ FFmpeg 相关 ------------------------

    def _start_ffmpeg(self):
        """
        启动 FFmpeg，将本地摄像头推流到 rtmp_url。
        """
        if self._ffmpeg_process and self._ffmpeg_process.poll() is None:
            # 日志调整: FFmpeg 已运行视为正常状态，不打印
            return

        cmd = [
            "ffmpeg",
            "-f",
            "v4l2",
            "-framerate",
            "30",
            "-video_size",
            "1280x720",
            "-i",
            "/dev/video0",
            "-c:v",
            "libx264",
            "-preset",
            "ultrafast",
            "-tune",
            "zerolatency",
            "-profile:v",
            "baseline",
            "-b:v",
            "1M",
            "-maxrate",
            "1M",
            "-bufsize",
            "2M",
            "-g",
            "10",
            "-keyint_min",
            "10",
            "-sc_threshold",
            "0",
            "-pix_fmt",
            "yuv420p",
            "-x264-params",
            "bframes=0",
            "-f",
            "flv",
            self.rtmp_url,
        ]
        # 日志调整: 不再打印完整命令，只在失败时打印异常

        try:
            self._ffmpeg_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
        except Exception as e:
            print(f"[CameraController] failed to start FFmpeg: {e}", file=sys.stderr)
            self._ffmpeg_process = None
            raise

    def _stop_ffmpeg(self):
        """
        停止 FFmpeg 推流。
        """
        if self._ffmpeg_process and self._ffmpeg_process.poll() is None:
            try:
                self._ffmpeg_process.terminate()
                self._ffmpeg_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    self._ffmpeg_process.kill()
                except Exception as e:
                    print(
                        f"[CameraController] failed to kill FFmpeg process: {e}",
                        file=sys.stderr,
                    )
            except Exception as e:
                print(
                    f"[CameraController] error when stopping FFmpeg: {e}",
                    file=sys.stderr,
                )
        self._ffmpeg_process = None

    # ------------------------ WebRTC Offer 相关 ------------------------

    def _handle_webrtc_offer(self, offer_sdp: str) -> str:
        """
        将浏览器的 Offer 发送给媒体服务器（SRS），
        拿到 Answer SDP 并返回。
        """
        payload = {
            "api": self.webrtc_api,
            "streamurl": self.webrtc_stream_url,
            "sdp": offer_sdp,
        }

        headers = {"Content-Type": "application/json"}
        try:
            resp = requests.post(
                self.webrtc_api,
                json=payload,
                headers=headers,
                timeout=10,
            )
        except Exception as e:
            print(
                f"[CameraController] failed to send offer to media server: {e}",
                file=sys.stderr,
            )
            raise

        # 日志调整: 不再打印正常 status/body，只在异常时 raise 并打印
        try:
            resp.raise_for_status()
        except Exception as e:
            print(
                f"[CameraController] media server HTTP error: {e}, "
                f"status={resp.status_code}, body={resp.text[:200]}",
                file=sys.stderr,
            )
            raise

        try:
            data = resp.json()
        except Exception as e:
            print(
                f"[CameraController] failed to parse media server JSON: {e}, "
                f"raw={resp.text[:200]}",
                file=sys.stderr,
            )
            raise

        answer_sdp = data.get("sdp", "")
        if not answer_sdp:
            msg = f"empty SDP from media server: {data}"
            print(f"[CameraController] {msg}", file=sys.stderr)
            raise RuntimeError(msg)

        return answer_sdp