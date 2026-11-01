"""라즈베리파이(주행 로봇) <-> 노트북 YOLO 서버 메시지 형식 (양쪽이 같이 사용).

한 메시지 = [헤더 길이 4바이트][데이터 길이 4바이트][JSON 헤더][데이터(JPEG 등, 없으면 0바이트)]
  서버 -> 로봇 (접속 직후 1번)   {"names": {"0": "STOP", ...}}
  로봇 -> 서버 (추론 요청)       {"imgsz": 320, "conf": 0.45, "classes": [0, 1, 2]} + JPEG
  서버 -> 로봇 (추론 결과)       {"boxes": [[cls, conf, x1, y1, x2, y2], ...], "ms": 추론 시간}
"""
import json
import struct

_HEAD = struct.Struct(">II")


def send_packet(sock, header, payload=b""):
    head = json.dumps(header).encode("utf-8")
    sock.sendall(_HEAD.pack(len(head), len(payload)) + head + payload)


def recv_packet(sock):
    head_len, payload_len = _HEAD.unpack(_recv_exact(sock, _HEAD.size))
    header = json.loads(_recv_exact(sock, head_len))
    payload = _recv_exact(sock, payload_len) if payload_len else b""
    return header, payload


def _recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("연결 끊김")
        buf += chunk
    return bytes(buf)
