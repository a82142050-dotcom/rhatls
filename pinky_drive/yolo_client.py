"""노트북 YOLO 서버에 추론 맡기기 (라즈베리파이 쪽).

연결이 안 되거나 / 답이 늦거나 / 모델이 다르면 None 을 돌려줌 -> 그 프레임은 라즈베리파이에서 직접 추론.
끊기면 YOLO_REMOTE_RETRY_SEC 마다 다시 연결 시도.
"""
import socket
import time

import cv2

from . import config as C
from .yolo_protocol import recv_packet, send_packet


class RemoteYolo:
    def __init__(self, host, port, local_names):
        self.host = host
        self.port = port
        self.local_names = local_names    # 라즈베리파이 모델 클래스 {id: NAME} -> 노트북 모델과 같아야 사용
        self.sock = None
        self.retry_at = 0.0
        self.warned = False

    def infer(self, frame, imgsz, conf, classes):
        """노트북에서 추론 -> [(cls, conf, x1, y1, x2, y2), ...]. 실패하면 None"""
        if self.sock is None and not self._connect():
            return None
        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), C.YOLO_REMOTE_JPEG_QUALITY])
        if not ok:
            return None
        try:
            send_packet(self.sock, {"imgsz": imgsz, "conf": conf, "classes": classes}, jpg.tobytes())
            header, _ = recv_packet(self.sock)
            return [tuple(b) for b in header["boxes"]]
        except (OSError, ValueError, KeyError) as e:   # socket.timeout / 연결 끊김 / 깨진 응답
            self._drop(f"노트북 추론 실패 ({type(e).__name__}: {e})")
            return None

    def _connect(self):
        now = time.time()
        if now < self.retry_at:
            return False
        self.retry_at = now + C.YOLO_REMOTE_RETRY_SEC
        try:
            sock = socket.create_connection((self.host, self.port), timeout=C.YOLO_REMOTE_TIMEOUT)
            sock.settimeout(C.YOLO_REMOTE_TIMEOUT)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # 작은 요청/응답 지연 없이
            header, _ = recv_packet(sock)                                 # 서버가 먼저 클래스 이름을 보냄
            names = {int(k): str(v).upper().strip() for k, v in header["names"].items()}
        except (OSError, ValueError, KeyError) as e:
            if not self.warned:
                print(f"[YOLO] 노트북 YOLO 서버 연결 안 됨 ({self.host}:{self.port}, {type(e).__name__})"
                      f" -> 라즈베리파이에서 추론, {C.YOLO_REMOTE_RETRY_SEC:g}초마다 다시 시도")
                self.warned = True
            return False

        if names != self.local_names:
            sock.close()
            self.retry_at = float("inf")      # 모델이 다르면 다시 시도 안 함
            print(f"[YOLO] 노트북 모델 클래스 {names} 가 라즈베리파이 모델 {self.local_names} 와 다름"
                  " -> 라즈베리파이에서만 추론")
            return False

        self.sock = sock
        self.warned = False
        print(f"[YOLO] 노트북 YOLO 서버 연결 -> 노트북에서 추론 ({self.host}:{self.port})")
        return True

    def _drop(self, why):
        print(f"[YOLO] {why} -> 라즈베리파이에서 추론, {C.YOLO_REMOTE_RETRY_SEC:g}초 뒤 다시 연결")
        try:
            self.sock.close()
        except OSError:
            pass
        self.sock = None
        self.retry_at = time.time() + C.YOLO_REMOTE_RETRY_SEC
