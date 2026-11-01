"""메인 제어 루프 + AUTO / RESET / 수동 조작 명령."""
import threading
import time
import traceback

import cv2

from . import config as C
from .autopilot import YOLO_STATES, Autopilot, Perception
from .lane import analyze_lane, check_goal_new_road
from .motion import Motion
from .overlay import render
from .state import State


MANUAL_CMDS = {
    "w": (C.MANUAL_SPEED, C.MANUAL_SPEED),
    "s": (-C.MANUAL_SPEED, -C.MANUAL_SPEED),
    "a": (-C.MANUAL_SPEED, C.MANUAL_SPEED),
    "d": (C.MANUAL_SPEED, -C.MANUAL_SPEED),
}


class Robot:
    def __init__(self, camera, motor, detector, odom, led=None):
        self.camera = camera
        self.detector = detector
        self.odom = odom                  # 직진 / 글자 정렬 에서만 사용
        self.s = State(odom)
        self.motion = Motion(motor, self.s, led)   # led: 주행 중 초록 / 정지 빨강
        self.autopilot = Autopilot(self.s, self.motion, notify=self.send_to_laptop)
        self.link = None     # 노트북(로봇팔) 소켓 연결 (main.py 에서 연결)

        self.stop_event = threading.Event()
        self._jpeg = None
        self._jpeg_lock = threading.Lock()
        self._goal_right_mask_was_on = False

    # ============================================================
    # 제어 루프
    # ============================================================
    def run(self):
        frame_no = 0
        fps_t, fps_n = time.time(), 0
        try:
            while not self.stop_event.is_set():
                t0 = time.time()
                try:
                    frame = self.camera.get_frame()
                    if frame is None:
                        time.sleep(0.02)
                        continue

                    # 웹 화면(그림 + JPEG)은 WEB_VIEW_EVERY 프레임마다 한 번만. 주행 제어는 매 프레임
                    frame_no += 1
                    combo = self.step(frame.copy(), draw=(frame_no % C.WEB_VIEW_EVERY == 0))
                    if combo is not None:
                        ok, jpg = cv2.imencode(".jpg", combo, [int(cv2.IMWRITE_JPEG_QUALITY), C.JPEG_QUALITY])
                        if ok:
                            with self._jpeg_lock:
                                self._jpeg = jpg.tobytes()

                    fps_n += 1                          # 화면 FPS 표시용 (1초마다 갱신)
                    if t0 - fps_t >= 1.0:
                        self.s.fps = fps_n / (t0 - fps_t)
                        fps_t, fps_n = t0, 0
                except Exception as e:
                    self._on_loop_error(e)

                # 예전 0.03초 고정 대기 대신: LOOP_MAX_FPS 보다 빠를 때만 남는 시간만큼 쉼
                rest = 1.0 / C.LOOP_MAX_FPS - (time.time() - t0)
                if rest > 0:
                    time.sleep(rest)
        finally:
            self._stop_motor()

    def _on_loop_error(self, e):
        """[안전] 제어 루프 오류: 모터 즉시 정지 + AUTO 해제, 루프는 계속 (웹 화면 / 수동 조작 유지)
        같은 오류가 매 프레임 반복되면 처음 한 번만 출력"""
        self._stop_motor()
        s = self.s
        s.auto_mode = False
        reason = f"{type(e).__name__}: {e}"
        if reason != s.stop_reason:
            print("[SAFETY] control loop error -> ROBOT STOP / AUTO OFF")
            traceback.print_exc()
            s.stop_reason = reason

    def _stop_motor(self):
        try:
            self.motion.stop()
        except Exception as e:
            print("[SAFETY] motor stop failed:", e)

    def step(self, frame, draw=True):
        """한 프레임 처리: 인식 -> 주행 -> 화면 이미지 반환 (draw=False 면 화면 안 그리고 None)"""
        s = self.s
        self.odom.update()          # 지난 프레임 이후 이동 거리 / 회전 각도 반영
        h, w = frame.shape[:2]
        roi_start = int(h * C.ROI_START_RATIO)
        roi = frame[roi_start:h, :]

        # ---------------- 중심선 ----------------
        
        # ---------------- 중심선 ----------------

        # GOAL 구간에서 CENTERLINE 상태로 진입한 뒤
        # 2.5초가 지났는지 확인
        goal_right_mask_on = (
            s.auto_mode
            and s.route_stage == "GOAL"
            and s.drive_state == "CENTERLINE"
            and s.elapsed() >= C.GOAL_RIGHT_MASK_DELAY_SEC
        )

        # 마스크가 처음 켜지는 순간
        if goal_right_mask_on and not self._goal_right_mask_was_on:

            # 이전에 잘못 따라가던 길의 조향 기억 초기화
            s.exit_err = None
            s.exit_lost_since = None
            s.last_error = 0

            print(
                f"[GOAL ROI] CENTERLINE "
                f"{C.GOAL_RIGHT_MASK_DELAY_SEC:g}s -> "
                f"RIGHT {C.GOAL_RIGHT_MASK_RATIO * 100:g}% OFF"
            )   

        self._goal_right_mask_was_on = goal_right_mask_on

        # GOAL CENTERLINE에서만 오른쪽 ROI 제거
        lane = analyze_lane(
            roi,
            roi_start,
            s.route_stage,
            exclude_right_ratio=(
                C.GOAL_RIGHT_MASK_RATIO
                if goal_right_mask_on else 0.0
            )
        )

        if lane.error is not None:
            s.last_error = lane.error


        s.narrow.update(lane.half_widths, lane.error, active=(
            s.auto_mode and s.initial_run_done
            and s.route_stage == "STATION" and s.drive_state == "CENTERLINE"))

        # ---------------- YOLO (처음 중심선 주행 이후) ----------------
        text, rejected = None, []
        if s.initial_run_done and s.drive_state in YOLO_STATES:
            try:
                text, rejected = self.detector.detect(roi, s.route_stage, y_offset=roi_start)
                s.yolo_info = f"{self.detector.source} {self.detector.infer_ms:.0f}ms"
            except Exception as e:
                print("YOLO ERROR:", e)

        confirmed = s.confirm.update(text, w,
                                     in_centerline=(s.drive_state == "CENTERLINE"),
                                     strict=(s.route_stage in C.EVENT2_STRICT_STAGES))

        # ---------------- GOAL 새 도로 확인 ----------------
        goal_lane_ready = False
        goal_lane_slope = None

        if (
            s.auto_mode
            and s.route_stage == "GOAL"
            and s.drive_state == "GOAL_LANE_SEARCH"
        ):
            goal_lane_ready, goal_lane_slope = check_goal_new_road(
                lane.mask,
                lane.points,
                lane.error
            )
        # ---------------- 주행 ----------------
        if time.time() < s.manual_until and s.manual_cmd is not None:
            self.motion.drive(*s.manual_cmd)
        elif not s.auto_mode:
            self.motion.stop()
        else:
            self.autopilot.step(Perception(text, w, h, lane.error, lane.offset_error, confirmed, goal_lane_ready, goal_lane_slope))

        # ---------------- 화면 (웹 디버그용, 주행엔 안 씀) ----------------
        return render(
            frame,
            lane,
            text,
            rejected,
            s,
            excluded_right_ratio=(
                C.GOAL_RIGHT_MASK_RATIO
                if goal_right_mask_on else 0.0
            )
        ) if draw else None

    def latest_jpeg(self):
        with self._jpeg_lock:
            return self._jpeg

    # ============================================================
    # 명령 (웹 버튼 / 키보드 / 노트북 소켓)
    # ============================================================
    def start_auto(self, source):
        """AUTO 시작 공통 (노트북 START 신호 / p 키)"""
        s = self.s
        if s.auto_mode:
            return
        if s.route_stage == "DONE":
            # 완주 후 다시 시작 (예: 로봇팔 프로그램만 재시작해서 START 가 다시 옴) -> 처음부터
            s.reset()
            print(f"[{source}] previous run DONE -> RESET for new run")
        s.auto_mode = True
        s.stop_reason = None
        if s.run_start_time is None:     # 이번 주행 처음 시작 (일시정지 후 재개는 이어서 잼)
            s.run_start_time = time.time()
        s.enter("CENTERLINE")       # 상태 시작 시각 / 오도메트리 시작 위치도 같이 기록
        if not s.initial_run_done:
            s.auto_start_dist = self.odom.dist
            print(f"[{source}] AUTO ON -> FIRST {C.CENTERLINE_RUN_CM:g}cm CENTERLINE")
        else:
            print(f"[{source}] AUTO ON -> NORMAL CENTERLINE")

    def send_to_laptop(self, msg, quiet=False):
        """로봇 -> 노트북(로봇팔) 메시지. 보냈으면 True (quiet=True 면 로그 안 찍음: 주기적 재전송용)"""
        if self.link is None:
            if not quiet:
                print(f"[핑키봇] 노트북 연결 없음 -> '{msg}' 못 보냄")
            return False
        return self.link.send(msg, quiet)

    def release_station(self, source):
        """STATION 정차 대기 해제 (로봇팔 임무 완료 메시지 / 스페이스바)"""
        s = self.s
        if s.drive_state == "STATION_WAIT":
            s.station_release = True
            print(f"[{source}] STATION release -> next")
        else:
            print(f"[{source}] release ignored (not waiting at STATION, state {s.drive_state})")

    def handle_key(self, key):
        s = self.s
        if key == "p":
            if s.auto_mode:
                s.auto_mode = False
                self.motion.stop()
                print("AUTO OFF")
            else:
                self.start_auto("KEY")

        elif key == "r":
            if self.link is not None:
                self.link.arm_done = False   # 새 주행: 로봇팔에 다시 접속 시도
            s.reset()
            self.motion.stop()
            print("RESET")

        elif key == "space":
            # 로봇팔 통신 대신 (단독 실행용): 멈춰 있으면 시작 (= START 신호) / STATION 정차 대기 중이면 출발 (= COMPLETE 신호)
            #   그 외 (주행 중)에는 정지
            if not s.auto_mode:
                self.start_auto("KEY")
            elif s.drive_state == "STATION_WAIT":
                self.release_station("KEY")
            else:
                s.auto_mode = False
                self.motion.stop()

        elif key == "stop":   # 웹 STOP 버튼: 항상 정지
            s.auto_mode = False
            self.motion.stop()

        elif key in MANUAL_CMDS:
            s.manual_cmd = MANUAL_CMDS[key]
            s.manual_until = time.time() + C.MANUAL_PULSE
