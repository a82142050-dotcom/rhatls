"""YOLO 글씨(STOP / STATION / GOAL) 검출 + 글씨 박스 판정.
노트북 YOLO 서버가 연결돼 있으면 노트북에서 추론, 아니면 라즈베리파이에서 직접 추론."""
import os
import time

import numpy as np
from ultralytics import YOLO

from . import config as C
from .yolo_client import RemoteYolo


def is_text_class(name):
    # [차단 2] 화살표 계열 이름은 무조건 제외
    return name in C.TEXT_CLASSES and not any(word in name for word in C.ARROW_BLOCK_WORDS)


def fully_visible(box, w):
    """박스 좌우가 화면 끝에서 FULL_MARGIN 이상 떨어져 있으면 글자 전체가 보이는 것"""
    x1, _, x2, _ = box["bbox"]
    return x1 >= C.FULL_MARGIN and x2 <= w - C.FULL_MARGIN


def seek_turn_dir(box, w):
    """박스가 잘린 쪽으로 회전 방향 결정 (오른쪽 잘림 -> 1, 왼쪽 잘림 -> -1)"""
    x1, _, x2, _ = box["bbox"]
    left_cut = x1 < C.FULL_MARGIN
    right_cut = x2 > w - C.FULL_MARGIN
    if right_cut != left_cut:
        return 1 if right_cut else -1
    return 1 if (x1 + x2) // 2 >= w // 2 else -1


class TextDetector:
    def __init__(self, model_path=C.MODEL_PATH, remote=None):
        """remote: (노트북 IP, 포트) 면 노트북 YOLO 서버 우선 사용. 라즈베리파이 모델은 대비용으로 항상 로드"""
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"{model_path} 파일이 없습니다. 이 코드와 같은 폴더에 넣어주세요.")

        print("Loading YOLO model...")
        self.model = YOLO(model_path)

        try:
            self.model.predict(
                np.zeros((C.YOLO_IMGSZ, C.YOLO_IMGSZ, 3), dtype=np.uint8),
                imgsz=C.YOLO_IMGSZ, conf=C.YOLO_CONF, verbose=False
            )
            print("YOLO warm-up done.")
        except Exception as e:
            print("YOLO warm-up failed:", e)

        names = self.model.names
        names = names.items() if isinstance(names, dict) else enumerate(names)
        self.class_names = {int(i): str(n).upper().strip() for i, n in names}
        print("YOLO classes:", self.class_names)

        # [차단 1] 허용 클래스 id 목록
        self.allowed_ids = [i for i, n in self.class_names.items() if is_text_class(n)] or None
        if self.allowed_ids is None:
            print("[WARN] STOP/STATION/GOAL class names not found in model -> classes filter off")
        print("YOLO allowed class ids (arrow blocked):", self.allowed_ids)

        self.remote = RemoteYolo(remote[0], remote[1], self.class_names) if remote else None
        self.source = "pi"        # 마지막 추론을 어디서 했는지 / 걸린 시간 (화면 표시용)
        self.infer_ms = 0.0

    def _infer(self, frame, min_conf):
        """[(cls, conf, x1, y1, x2, y2), ...] - 노트북 서버가 되면 노트북, 아니면 라즈베리파이"""
        t0 = time.time()
        if self.remote is not None:
            boxes = self.remote.infer(frame, C.YOLO_IMGSZ, min_conf, self.allowed_ids)
            if boxes is not None:
                self.source, self.infer_ms = "laptop", (time.time() - t0) * 1000
                return boxes

        results = self.model(frame, imgsz=C.YOLO_IMGSZ, conf=min_conf,
                             classes=self.allowed_ids, verbose=False)
        self.source, self.infer_ms = "pi", (time.time() - t0) * 1000
        if not results or results[0].boxes is None:
            return []
        return [(int(b.cls[0]), float(b.conf[0]), *b.xyxy[0].tolist()) for b in results[0].boxes]

    def detect(self, frame, route_stage, y_offset=0):
        """이번 구간에서 찾는 글씨 중 화면 가장 아래(가장 가까운) 박스 하나.
        반환: (박스 dict 또는 None, 모양 때문에 버린 박스 목록)"""
        # 구간별 신뢰도를 모델 호출에도 적용 (GOAL처럼 0.45보다 낮은 값도 통과되게)
        min_conf = C.STAGE_MIN_CONF.get(route_stage, C.YOLO_CONF)
        expected_class = C.STAGE_TARGET.get(route_stage)
        rejected = []

        candidates = []
        for cls_id, conf, bx1, by1, bx2, by2 in self._infer(frame, min_conf):
            name = self.class_names[int(cls_id)]
            conf = float(conf)

            if conf < min_conf or not is_text_class(name):
                continue
            if expected_class is not None and name != expected_class:
                continue

            x1, y1, x2, y2 = int(bx1), int(by1), int(bx2), int(by2)
            y1, y2 = y1 + y_offset, y2 + y_offset

            # [차단 3] 글씨처럼 가로로 길지 않으면(화살표 모양) 버림
            aspect = max(1, x2 - x1) / max(1, y2 - y1)
            if aspect < C.TEXT_MIN_ASPECT.get(name, 1.5):
                rejected.append((x1, y1, x2, y2, name, aspect))
                continue

            candidates.append({
                "type": name,
                "conf": conf,
                "bbox": (x1, y1, x2, y2),
                "center": ((x1 + x2) // 2, (y1 + y2) // 2),
            })

        best = max(candidates, key=lambda z: z["bbox"][3]) if candidates else None
        return best, rejected


class TextConfirm:
    """중심선 주행 중 글씨가 N 프레임 연속 보여야 이벤트 실행 (화살표 순간 오인식 무시)"""

    def __init__(self):
        self.hits = 0
        self.full_hits = 0      # 확인 중 글자 전체가 보인 프레임 수

    def reset(self):
        self.hits = 0
        self.full_hits = 0

    def update(self, text, w, in_centerline, strict):
        """매 프레임 호출. 확인 완료면 True"""
        if in_centerline and text is not None:
            self.hits += 1
            self.full_hits = self.full_hits + 1 if fully_visible(text, w) else 0
        elif in_centerline and not strict:
            # 화살표 없는 구간: 한 프레임 놓쳐도 1만 깎음
            self.hits = max(0, self.hits - 1)
        else:
            self.reset()

        need = C.EVENT2_CONFIRM_FRAMES
        if text is not None:
            need = C.EVENT2_CONFIRM_FRAMES_BY_TYPE.get(text["type"], C.EVENT2_CONFIRM_FRAMES)
        return self.hits >= need
