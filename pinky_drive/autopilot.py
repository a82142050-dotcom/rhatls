"""자동주행 상태 머신. 상태 하나 = 메서드 하나.

CENTERLINE ─(처음 215cm)─────────> SEARCH_TEXT ──(글자 전체 보임)──┐
    │                              └─(8초 못 찾음)─> 안전 정지     │
    ├─(글씨 확인, 잘림)──> TEXT_SEEK_FULL ──(전체 보임)──────────> APPROACH_TEXT
    │                     (멈춰서 보기) <-> TEXT_SEEK_TURN (SEEK_STEP_DEG 만큼 회전)
    ├─(글씨 확인, 전체)─────────────────────────────────────────> APPROACH_TEXT
    ├─(STATION 구간 도로 좁아짐)─> STATION_NARROW_FORWARD
    │       -> STATION_TURN_SEARCH (멈춰서 보기) <-> STATION_SEARCH_TURN (오른쪽으로 STEP 만큼 회전)
    │          ─(찾음)─> TEXT_SEEK_FULL / ─(MAX_DEG 까지 못 찾음)─> CENTERLINE
    └─(GOAL 확인)─> GOAL_APPROACH

APPROACH_TEXT -> DRIVE_TEXT -> FORWARD_AFTER_TEXT
    -> STATION 이면 STATION_WAIT (로봇팔에 STATION 보내고, 임무 완료 메시지 / 스페이스바 올 때까지 정지)
         -> STOP2 구간 CENTERLINE
    -> STOP1 / STOP2 이면 STOP_AFTER_TEXT(3초 정지)
         -> STOP1 이면 STATION 구간 CENTERLINE
         -> STOP2 이면 FINAL_FORWARD -> FINAL_RIGHT_TURN -> CENTERLINE(GOAL 구간)

GOAL_APPROACH -> GOAL_FINAL_FORWARD -> DONE

오도메트리(거리 cm / 각도 deg)로 끝나는 동작:
  거리 = 처음 중심선 주행(CENTERLINE_RUN_CM)
         STATION_NARROW_FORWARD, FORWARD_AFTER_TEXT, FINAL_FORWARD, GOAL_FINAL_FORWARD  (m.straight_for)
  각도 = TEXT_SEEK_TURN, STATION_SEARCH_TURN, FINAL_RIGHT_TURN                       (m.turn_for)
초 단위로 남은 것 = 3초 정지, 못 찾을 때 포기하는 시간(타임아웃).
"""
import threading
from .goal_sound_police import play_goal_song

import time
from collections import namedtuple

from . import config as C
from .detector import fully_visible, seek_turn_dir


# 한 프레임에서 상태 머신이 보는 입력
Perception = namedtuple("Perception", [
    "text",           # 이번 구간 글씨 박스 (없으면 None)
    "w", "h",         # 화면 크기
    "error",          # 도로 중심선 오차
    "offset_error",   # 우측 오프셋 중심선 오차
    "confirmed", 
    "goal_lane_ready",
    "goal_lane_slope",# 중심선 주행 중 글씨 연속 확인 완료
])

# YOLO를 돌리는 주행 상태
#   TEXT_SEEK_TURN / STATION_SEARCH_TURN 은 일부러 뺌: 도는 중엔 YOLO 결과를 안 쓰고, 루프가 빨라야 각도에서 정확히 멈춤
YOLO_STATES = {
    "CENTERLINE", "SEARCH_TEXT", "TEXT_SEEK_FULL", "STATION_TURN_SEARCH",
    "APPROACH_TEXT", "DRIVE_TEXT", "GOAL_APPROACH",
}


class Autopilot:
    def __init__(self, state, motion, notify=lambda msg, quiet=False: False):
        self.s = state
        self.m = motion
        self.notify = notify   # 로봇팔(노트북)에 메시지 보내기 -> 보냈으면 True
        self.goal_lane_hits = 0
        self.goal_lane_steps = 0
        self.handlers = {
            "CENTERLINE": self._centerline,
            "SEARCH_TEXT": self._search_text,
            "STATION_NARROW_FORWARD": self._station_narrow_forward,
            "STATION_TURN_SEARCH": self._station_turn_search,
            "STATION_SEARCH_TURN": self._station_search_turn,
            "TEXT_SEEK_FULL": self._text_seek_full,
            "TEXT_SEEK_TURN": self._text_seek_turn,
            "APPROACH_TEXT": self._approach_text,
            "DRIVE_TEXT": self._drive_text,
            "FORWARD_AFTER_TEXT": self._forward_after_text,
            "STATION_WAIT": self._station_wait,
            "STOP_AFTER_TEXT": self._stop_after_text,
            "FINAL_FORWARD": self._final_forward,
            "FINAL_RIGHT_TURN": self._final_right_turn,
            "GOAL_LANE_SEARCH": self._goal_lane_search,
            "GOAL_LANE_TURN": self._goal_lane_turn,
            "GOAL_APPROACH": self._goal_approach,
            "GOAL_FINAL_FORWARD": self._goal_final_forward,
            "DONE": self._done,
        }

    def step(self, p):
        self.handlers[self.s.drive_state](p)

    # ============================================================
    # 공통 전환
    # ============================================================
    def _back_to_centerline(self, msg=None):
        self.s.current_text_type = None
        self.s.enter("CENTERLINE", msg)

    def _start_text_seek(self, text, w):
        """STOP / STATION: 박스 전체가 화면에 들어올 때까지 회전 정렬"""
        s = self.s
        self.m.stop()
        s.seek_dir = seek_turn_dir(text, w)
        s.seek_start_yaw = s.odom.yaw if s.odom is not None else 0.0
        s.seek_steps = 0
        self._enter_seek_look(f"[YOLO] {s.current_text_type} detected{self._leg_text()} -> TEXT_SEEK_FULL "
                              f"({'right' if s.seek_dir > 0 else 'left'} cut)")

    def _enter_seek_look(self, msg=None):
        """정렬: 멈춰서 보기 (TEXT_SEEK_FULL) 시작"""
        s = self.s
        s.text_full_count = 0
        s.seek_frames = 0
        s.seek_misses = 0
        s.enter("TEXT_SEEK_FULL", msg)

    def _start_goal_approach(self, msg):
        self.s.goal_seen_near = False
        self.s.goal_last_seen = time.time()
        self.s.enter("GOAL_APPROACH", msg)

    def _safe_stop(self, reason):
        """모터 정지 + AUTO 해제. AUTO 다시 누르면 중심선 주행(YOLO 켜진 상태)부터 재개"""
        self.m.stop()
        self.s.auto_mode = False
        self.s.stop_reason = reason
        print(f"[SAFETY] {reason} -> ROBOT STOP / AUTO OFF")

    # ============================================================
    # CENTERLINE: 기본 주행 + 이벤트 감시
    # ============================================================
    def _centerline(self, p):
        s, m = self.s, self.m
        if s.auto_start_dist is None:
            s.auto_start_dist = s.odom.dist

        # [이벤트 1] 시작 후 CENTERLINE_RUN_CM 주행 -> 정지 후 제자리 회전하며 글씨 탐색
        if not s.initial_run_done and s.run_dist() >= C.CENTERLINE_RUN_CM:
            m.stop()
            s.initial_run_done = True
            s.text_full_count = 0
            s.text_candidate_type = None
            s.enter("SEARCH_TEXT", f"[ODOM] first {s.run_dist():.0f}cm centerline done -> SEARCH_TEXT")

        # [이벤트 2] 글씨가 연속 프레임으로 확인되면 글씨 처리 시작
        elif p.text is not None and p.confirmed and self._text_in_range(p):
            self._on_text_confirmed(p)

        # [STATION 앞] 흰 도로가 갑자기 좁아지고 중심선이 옆으로 쏠림 (STOP1 부터 NARROW_MIN_CM 이후만)
        elif s.narrow.trigger and self._narrow_in_range():
            s.narrow.hits = 0
            s.current_text_type = "STATION"
            s.enter("STATION_NARROW_FORWARD",
                    f"[STATION] road narrowed (W {s.narrow.road_w:.0f} / BASE {s.narrow.base():.0f})"
                    f"{self._leg_text()} -> forward {C.NARROW_FORWARD_CM:.0f}cm more")

        # [STATION 앞] 좁아짐을 못 잡았어도 STOP1 부터 STATION_FORCE_SEARCH_CM 가면 멈추고 찾기 (한 번만)
        elif self._force_station_search():
            s.station_force_done = True
            s.current_text_type = "STATION"
            self._start_station_search(f"[STATION] no narrowing until{self._leg_text()}"
                                       f" (FORCE {C.STATION_FORCE_SEARCH_CM:g}cm) -> stop, turn right to find STATION")

        # 주행
        else:
            # [이벤트 3] STOP2 구간은 우측 오프셋 중심선을 따라감
            err = p.offset_error if s.route_stage in C.RIGHT_OFFSET_STAGES else p.error

            if s.route_stage in C.GENTLE_STAGES:
                # ★ STOP2 인식하러 갈 때: 고개를 확 돌리지 않게 부드럽게 (오도메트리 안 씀)
                #   STOP이 보이기 시작하면 감속해서 확인 프레임을 더 확보
                m.follow_gentle(err, slow=(p.text is not None))
            elif s.route_stage in C.EXIT_STAGES:
                m.follow_exit(err)          # GOAL 구간: 평활화
            else:
                m.follow_centerline(err)    # 기본

    def _on_text_confirmed(self, p):
        s = self.s
        s.current_text_type = p.text["type"]
        full_hits = s.confirm.full_hits
        s.confirm.reset()

        if s.current_text_type == "GOAL":
            self._start_goal_approach("[YOLO] GOAL detected -> GOAL_APPROACH")
        elif fully_visible(p.text, p.w) and full_hits >= C.FAST_FULL_FRAMES:
            # 이미 글자 전체가 보였음 -> 멈춰서 다시 확인하지 않고 바로 접근
            s.enter("APPROACH_TEXT", f"[YOLO] {s.current_text_type} already full{self._leg_text()} -> APPROACH_TEXT (fast)")
        else:
            self._start_text_seek(p.text, p.w)

    # ============================================================
    # SEARCH_TEXT: 처음 중심선 주행 직후 제자리 회전하며 글자 전체가 보일 때까지 탐색
    # ============================================================
    def _search_text(self, p):
        s, m, t = self.s, self.m, p.text
        full_target = None

        if t is not None and fully_visible(t, p.w):
            if s.text_candidate_type == t["type"]:
                s.text_full_count += 1
            else:
                s.text_candidate_type = t["type"]
                s.text_full_count = 1
            if s.text_full_count >= C.FULL_STABLE_FRAMES:
                full_target = t
        else:
            s.text_full_count = 0
            s.text_candidate_type = None

        if full_target is None:
            if s.elapsed() >= C.SEARCH_TEXT_TIMEOUT_SEC:
                target = C.STAGE_TARGET.get(s.route_stage, "TEXT")
                self._safe_stop(f"{target} not found in {C.SEARCH_TEXT_TIMEOUT_SEC:.1f}s search")
            else:
                m.spin(C.TURN_SPEED)
            return

        m.stop()
        s.current_text_type = full_target["type"]
        if s.current_text_type == "GOAL":
            self._start_goal_approach("[YOLO] GOAL full -> GOAL_APPROACH")
        else:
            s.enter("APPROACH_TEXT", f"[YOLO] {s.current_text_type} full -> APPROACH_TEXT")

    # ============================================================
    # STATION 앞 도로 좁아짐: 조금 더 직진 -> 멈춤 -> 오른쪽으로 "STEP 만큼 돌고 -> 멈춰서 보기" 반복
    #   예전: "0.12초 돌고 0.15초 멈춤" 시간 펄스 5초
    # ============================================================
    def _leg_text(self):
        """로그용 ' (STOP1 +123cm)'. STATION 구간이 아니거나 STOP1 을 안 거쳤으면 빈 글자"""
        leg = self.s.station_leg_cm()
        return "" if leg is None or self.s.route_stage != "STATION" else f" (STOP1 +{leg:.0f}cm)"

    def _narrow_in_range(self):
        """좁아짐을 STATION 신호로 인정할 거리인지 (NARROW_MIN_CM 전이면 조명 오판으로 보고 무시)"""
        s = self.s
        leg = s.station_leg_cm()
        if C.NARROW_MIN_CM <= 0 or leg is None or leg >= C.NARROW_MIN_CM:
            return True
        s.narrow.hits = 0
        if time.time() - s.narrow_ignored_at >= 1.0:   # 좁아 보이는 동안 매 프레임 찍히지 않게 1초에 한 번
            s.narrow_ignored_at = time.time()
            print(f"[STATION] road narrowed at{self._leg_text()} < MIN {C.NARROW_MIN_CM:g}cm -> ignore")
        return False

    def _text_in_range(self, p):
        """달리면서 본 글씨를 바로 처리할지. STATION 구간에서 STOP1 부터 STATION_YOLO_MIN_CM 전이면 무시
        (일찍 보고 꺾으면 안쪽으로 파고들어 정차 위치가 쏠림 -> 좁아짐 / 강제 찾기 지점에서 멈춰서 찾게)"""
        s = self.s
        leg = s.station_leg_cm()
        if (C.STATION_YOLO_MIN_CM <= 0 or leg is None or s.route_stage != "STATION"
                or p.text["type"] != "STATION" or leg >= C.STATION_YOLO_MIN_CM):
            return True
        s.confirm.reset()
        if time.time() - s.yolo_ignored_at >= 1.0:   # 보이는 동안 매 프레임 찍히지 않게 1초에 한 번
            s.yolo_ignored_at = time.time()
            print(f"[STATION] YOLO seen at{self._leg_text()} < {C.STATION_YOLO_MIN_CM:g}cm -> ignore (keep centerline)")
        return False

    def _force_station_search(self):
        s = self.s
        leg = s.station_leg_cm()
        return (C.STATION_FORCE_SEARCH_CM > 0 and leg is not None and not s.station_force_done
                and s.route_stage == "STATION" and leg >= C.STATION_FORCE_SEARCH_CM)

    def _start_station_search(self, msg):
        """멈추고 오른쪽으로 '한 스텝 돌고 -> 멈춰서 보기' 반복 시작"""
        s = self.s
        s.station_search_yaw = s.odom.yaw
        s.station_look_frames = 0
        s.enter("STATION_TURN_SEARCH", msg)

    def _station_narrow_forward(self, p):
        # ★ 오도메트리 직진: NARROW_FORWARD_CM 만큼 (예전: 0.85초)
        if self.m.straight_for(C.NARROW_FORWARD_CM, C.NARROW_FORWARD_SPEED):
            self._start_station_search("[STATION] forward done -> stop, turn right step by step to find STATION")

    def _station_turn_search(self, p):
        """멈춰서 YOLO 로 STATION 찾기 -> 못 찾으면 오른쪽으로 한 스텝 더"""
        s = self.s
        self.m.stop()
        if p.text is not None:
            # 찾음 -> 기존 정렬 과정 그대로 (잘렸으면 잘린 쪽으로 정렬, 전체면 접근)
            s.current_text_type = "STATION"
            self._start_text_seek(p.text, p.w)
            return

        s.station_look_frames += 1
        if s.station_look_frames < C.STATION_SEARCH_LOOK_FRAMES:
            return

        total = s.odom.yaw - s.station_search_yaw
        if total + C.STATION_SEARCH_STEP_DEG > C.STATION_SEARCH_MAX_DEG:
            s.narrow.cooldown_until = time.time() + C.NARROW_COOLDOWN_SEC
            self._back_to_centerline(f"[STATION] not found in {total:+.0f}deg -> back to CENTERLINE")
            return
        s.enter("STATION_SEARCH_TURN")

    def _station_search_turn(self, p):
        """오른쪽으로 STATION_SEARCH_STEP_DEG 만큼만 돌고 멈춤 -> 다시 멈춰서 보기"""
        if self.m.turn_for(C.STATION_SEARCH_STEP_DEG, C.STATION_SEARCH_SPEED):
            self.s.station_look_frames = 0
            self.s.enter("STATION_TURN_SEARCH")

    # ============================================================
    # ★ 정렬 (오도메트리): TEXT_SEEK_FULL(멈춰서 보기) <-> TEXT_SEEK_TURN(SEEK_STEP_DEG 만큼만 회전)
    #   예전: "0.12초 돌고 0.15초 멈춤" 시간 펄스
    #         -> YOLO 때문에 루프 한 바퀴가 0.12초보다 길면 명령이 다음 프레임까지 유지돼서 한 번에 20도 넘게 돎
    #   지금: 각도로 끊어서 "8도 돌고 -> 멈춰서 보고" 반복
    # ============================================================
    def _text_seek_full(self, p):
        """멈춰서 YOLO 결과를 보고 다음 행동 결정"""
        s, t = self.s, p.text
        self.m.stop()
        s.seek_frames += 1
        if s.seek_frames <= C.SEEK_SETTLE_FRAMES:      # 멈춘 직후 프레임은 흔들림 -> 판단 안 함
            return

        total = s.odom.yaw - s.seek_start_yaw          # 정렬 시작 후 총 회전 각도

        # 안 보임: 몇 프레임 더 기다려 보고, 그래도 없으면 중심선 추종으로 복귀 (돌면서 찾지 않음)
        if t is None:
            s.text_full_count = 0
            s.seek_misses += 1
            if s.seek_misses >= C.SEEK_MISS_FRAMES:
                self._back_to_centerline(f"[ALIGN] {s.current_text_type} lost ({total:+.0f}deg) -> back to CENTERLINE")
            return
        s.seek_misses = 0

        # 전체 보임이 연속 FULL_STABLE_FRAMES 프레임 -> 접근
        if fully_visible(t, p.w):
            s.text_full_count += 1
            if s.text_full_count >= C.FULL_STABLE_FRAMES:
                s.text_full_count = 0
                s.enter("APPROACH_TEXT",
                        f"[ALIGN] {s.current_text_type} all letters visible ({total:+.0f}deg) -> APPROACH_TEXT")
            return
        s.text_full_count = 0

        # 잘려 있음 -> 잘린 쪽으로 한 스텝 더
        new_dir = seek_turn_dir(t, p.w)
        flipped = s.seek_steps > 0 and new_dir != s.seek_dir                 # 왔다 갔다 = 글씨가 화면보다 큼
        too_far = abs(total + new_dir * C.SEEK_STEP_DEG) > C.SEEK_MAX_DEG     # 더 돌면 너무 많이 돎
        if flipped or too_far:
            why = "left-right flip" if flipped else f"max {C.SEEK_MAX_DEG}deg"
            s.enter("APPROACH_TEXT", f"[ALIGN] {why} ({total:+.0f}deg) -> APPROACH_TEXT as it is")
            return

        s.seek_dir = new_dir
        s.seek_steps += 1
        s.enter("TEXT_SEEK_TURN")

    def _text_seek_turn(self, p):
        """SEEK_STEP_DEG 만큼만 돌고 멈춤 -> 다시 멈춰서 보기"""
        if self.m.turn_for(self.s.seek_dir * C.SEEK_STEP_DEG, C.SEEK_STEP_SPEED):
            self._enter_seek_look()

    # ============================================================
    # STOP / STATION 글씨 처리: 접근 -> 바닥까지 -> 직진 -> 정지(STATION 은 로봇팔 대기) -> 다음 구간
    # ============================================================
    def _approach_text(self, p):
        """박스 중심 맞추며 접근, 박스 하단이 화면 60%까지 오면 DRIVE_TEXT"""
        self.m.follow_text(p.text, p.w)
        if p.text is not None and p.text["bbox"][3] >= int(p.h * C.TEXT_ALIGN_TRIGGER_RATIO):
            self.s.enter("DRIVE_TEXT",
                         f"[YOLO] {p.text['type']} reached {C.TEXT_ALIGN_TRIGGER_RATIO:.0%} -> DRIVE_TEXT")

    def _drive_text(self, p):
        """박스 하단이 화면 바닥에 닿을 때까지 따라감"""
        s = self.s
        if p.text is not None and p.text["bbox"][3] >= int(p.h * C.TEXT_BOTTOM_TRIGGER_RATIO):
            self.m.stop()
            s.enter("FORWARD_AFTER_TEXT",
                    f"[YOLO] {s.current_text_type} bbox bottom reached -> forward {C.FORWARD_AFTER_TEXT_CM:.0f}cm")
        else:
            self.m.follow_text(p.text, p.w)

    def _forward_after_text(self, p):
        # ★ 오도메트리 직진: FORWARD_AFTER_TEXT_CM 만큼 (예전: 3초)
        s = self.s
        if not self.m.straight_for(C.FORWARD_AFTER_TEXT_CM, C.TEXT_FOLLOW_SPEED):
            return

        if s.route_stage == "STATION":
            s.station_msg_sent = False
            s.station_release = False     # 도착 전에 온 신호는 무시하고 새로 기다림
            s.enter("STATION_WAIT",
                    f"[STATION] forward {s.moved_cm():.0f}cm done{self._leg_text()} -> send STATION to robot arm,"
                    f" wait for mission complete (space = next)")
            if self.notify(C.MSG_STATION_ARRIVED):    # 대기 상태로 바꾼 뒤에 보냄 (답이 빨리 와도 받게)
                s.station_msg_sent = True
            s.station_sent_at = time.time()
        else:
            s.enter("STOP_AFTER_TEXT",
                    f"[TEXT] forward {s.moved_cm():.0f}cm done -> stop {C.STOP_AFTER_TEXT_SEC:.1f}s")

    def _station_wait(self, p):
        """로봇팔 임무 완료 메시지(또는 스페이스바)가 올 때까지 정지"""
        s = self.s
        self.m.stop()
        if not s.station_release:
            # 로봇팔은 스캔 중에 받은 STATION 을 무시함 -> 완료 신호가 올 때까지 주기적으로 다시 보냄
            if time.time() - s.station_sent_at >= C.STATION_RESEND_SEC:
                s.station_sent_at = time.time()
                if self.notify(C.MSG_STATION_ARRIVED, quiet=True):
                    s.station_msg_sent = True
            return

        s.station_release = False
        prev_stage = s.route_stage
        s.route_stage = C.NEXT_STAGE[prev_stage]
        self._back_to_centerline(f"[ROUTE] {prev_stage} done -> CENTERLINE / SEARCH {s.route_stage}")

    def _stop_after_text(self, p):
        s = self.s
        self.m.stop()
        if s.elapsed() < C.STOP_AFTER_TEXT_SEC:
            return

        prev_stage = s.route_stage
        if prev_stage in C.NEXT_STAGE:
            s.route_stage = C.NEXT_STAGE[prev_stage]
            if s.route_stage == "STATION" and s.odom is not None:
                s.station_leg_start = s.odom.dist   # STOP1 출발 위치 -> 여기서부터 거리로 STATION 찾는 범위 판단
                s.station_force_done = False
            self._back_to_centerline(f"[ROUTE] {prev_stage} done -> CENTERLINE / SEARCH {s.route_stage}")
        elif prev_stage == "STOP2":
            s.current_text_type = None
            s.enter("FINAL_FORWARD", f"[ROUTE] STOP2 done -> FINAL_FORWARD {C.FINAL_STOP_FORWARD_CM:.0f}cm")
        else:
            self._back_to_centerline()

    # ============================================================
    # STOP2 이후: 직진 -> 우회전 -> GOAL 구간 중심선
    # ============================================================
    def _final_forward(self, p):
        # ★ 오도메트리 직진: FINAL_STOP_FORWARD_CM 만큼 (예전: 3초)
        if self.m.straight_for(C.FINAL_STOP_FORWARD_CM, C.TEXT_FOLLOW_SPEED):
            self.s.enter("FINAL_RIGHT_TURN",
                         f"[ROUTE] final forward {self.s.moved_cm():.0f}cm done"
                         f" -> right turn {C.FINAL_TURN_DEG}deg")

    def _final_right_turn(self, p):
        """기존 150도 우회전 후 새 도로 확인 상태로 전환"""
        s = self.s

        if not self.m.turn_for(
            C.FINAL_TURN_DEG,
            C.FINAL_TURN_SPEED
        ):
            return

        self.goal_lane_hits = 0
        self.goal_lane_steps = 0

        s.route_stage = "GOAL"
        s.exit_err = None
        s.exit_lost_since = None

        s.enter(
            "GOAL_LANE_SEARCH",
            "[GOAL] right turn done -> check new road"
        )

    def _goal_lane_search(self, p):
        """정지하여 도로 확인, 실패하면 오른쪽으로 30도 추가 회전"""
        s = self.s
        self.m.stop()

        # 회전 직후 카메라 안정화
        if s.elapsed() < C.GOAL_LANE_SETTLE_SEC:
            self.goal_lane_hits = 0
            return

        # 새 도로가 검출되면 연속 확인
        if p.goal_lane_ready:
            self.goal_lane_hits += 1

            if self.goal_lane_hits >= C.GOAL_LANE_CONFIRM_FRAMES:
                s.exit_err = None
                s.exit_lost_since = None

                s.enter(
                    "CENTERLINE",
                    f"[GOAL] new road confirmed "
                    f"(slope={p.goal_lane_slope}) "
                    f"-> CENTERLINE"
                )
                return
        else:
            self.goal_lane_hits = 0

        # 아직 확인 시간이 남았으면 계속 정지
        if s.elapsed() < C.GOAL_LANE_LOOK_SEC:
            return

        # 최대 탐색 횟수 초과 시 안전 정지
        if self.goal_lane_steps >= C.GOAL_LANE_MAX_STEPS:
            self._safe_stop(
                "[GOAL] new road not found "
                "after maximum search -> AUTO OFF"
            )
            return

        # 오른쪽으로 30도 추가 회전
        self.goal_lane_steps += 1

        s.enter(
            "GOAL_LANE_TURN",
            f"[GOAL] road not found "
            f"(slope={p.goal_lane_slope}) "
            f"-> right +{C.GOAL_LANE_STEP_DEG}deg "
            f"({self.goal_lane_steps}/{C.GOAL_LANE_MAX_STEPS})"
        )

    def _goal_lane_turn(self, p):
        """오른쪽으로 30도 회전 후 다시 정지하여 도로 확인"""
        s = self.s

        if not self.m.turn_for(
            C.GOAL_LANE_STEP_DEG,
            C.GOAL_LANE_TURN_SPEED
        ):
            return

        self.goal_lane_hits = 0

        s.enter(
            "GOAL_LANE_SEARCH",
            "[GOAL] +30deg done -> check new road"
        )

    # ============================================================
    # GOAL: 박스 따라 직진 (안 보여도 멈추지 않음) -> 사라지면 더 직진 -> 완전 정지
    #   GOAL 인식 후는 직진만 하므로 GOAL_SPEED (글씨 따라가기보다 20% 빠르게)
    # ============================================================
    def _goal_approach(self, p):
        s = self.s
        self.m.follow_text(p.text, p.w, speed=C.GOAL_SPEED)

        if p.text is not None:
            s.goal_last_seen = time.time()
            if p.text["bbox"][3] >= int(p.h * C.GOAL_NEAR_RATIO):
                s.goal_seen_near = True
            return

        missing = time.time() - s.goal_last_seen
        if s.goal_seen_near and missing >= C.GOAL_MISS_TIMEOUT:
            s.enter("GOAL_FINAL_FORWARD", f"[GOAL] box gone -> forward {C.GOAL_FINAL_FORWARD_CM:.0f}cm more")
        elif missing >= C.GOAL_GIVEUP_SEC:
            s.enter("GOAL_FINAL_FORWARD",
                    f"[GOAL] lost {C.GOAL_GIVEUP_SEC:.1f}s -> forward {C.GOAL_FINAL_FORWARD_CM:.0f}cm more")

    def _goal_final_forward(self, p):
        s = self.s
        # ★ 오도메트리 직진: GOAL_FINAL_FORWARD_CM 만큼 (예전: 7초)
        if not self.m.straight_for(C.GOAL_FINAL_FORWARD_CM, C.GOAL_SPEED):
            return

        moved = s.moved_cm()
        s.run_end_time = time.time()
        s.route_stage = "DONE"
        s.auto_mode = False
        s.enter("DONE", f"[GOAL] final forward {moved:.0f}cm done -> ROBOT STOP / DONE"
                        f" | 총 소요 시간 {s.run_time():.1f}s")
        
        # ★ GOAL 완주 후 모터 정지
        self.m.stop()

        # ★ 완주 멜로디 1회 재생
        threading.Thread(
            target=play_goal_song,
            daemon=True
        ).start()

    def _done(self, p):
        self.m.stop()
