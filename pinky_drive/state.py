"""로봇 전체 상태 (주행 상태 머신 + 코스 진행 + 감지기 기억값)."""
import time

from .detector import TextConfirm
from .lane import RoadNarrowDetector


class State:
    def __init__(self, odom=None):
        self.manual_until = 0.0
        self.manual_cmd = None
        self.link_status = "connecting"   # 노트북 소켓 연결 상태 (RESET 해도 유지)
        self.yolo_info = ""               # 마지막 YOLO 추론 위치 / 시간 (화면 표시용, 예: "laptop 18ms")
        self.fps = 0.0                    # 제어 루프 초당 프레임 수 (화면 표시용)
        self.odom = odom                  # odometry.Odometry (직진 / 정렬 에서만 사용, reset 해도 유지)
        self.reset()

    def reset(self):
        self.auto_mode = False
        self.drive_state = "CENTERLINE"   # autopilot.py 의 상태 이름
        self.route_stage = "STOP1"        # STOP1 -> STATION -> STOP2 -> GOAL -> DONE
        self.state_start_time = 0.0
        self.last_error = 0

        self.run_start_time = None        # 이번 주행 시작 시각 (처음 AUTO 켠 순간) -> 화면 TIME
        self.run_end_time = None          # 완주(DONE) 시각 -> 화면에 GOAL 시간으로 고정
        self.auto_start_dist = None       # 처음 중심선 주행을 시작한 위치 (오도메트리 cm)
        self.initial_run_done = False     # 처음 CENTERLINE_RUN_CM 주행 끝남 (그 뒤부터 YOLO 사용)

        self.current_text_type = None
        self.text_candidate_type = None
        self.text_full_count = 0

        self.seek_dir = 1                 # 1 = 오른쪽 회전, -1 = 왼쪽 회전
        self.mark = (0.0, 0.0)            # 지금 상태에 들어올 때의 오도메트리 (거리 cm, 각도 deg)

        # 정렬 (TEXT_SEEK_FULL <-> TEXT_SEEK_TURN)
        self.seek_start_yaw = 0.0         # 정렬 시작할 때 방향
        self.seek_steps = 0               # 지금까지 돈 스텝 수
        self.seek_frames = 0              # 멈춰서 본 프레임 수
        self.seek_misses = 0              # 글씨가 연속으로 안 보인 프레임 수

        # STATION 찾기 회전 (STATION_TURN_SEARCH <-> STATION_SEARCH_TURN)
        self.station_search_yaw = 0.0     # 찾기 시작할 때 방향
        self.station_look_frames = 0      # 이번 스텝에서 멈춰서 본 프레임 수
        self.station_leg_start = None     # STOP1 출발할 때 오도메트리 거리 (cm) -> STOP1 부터 간 거리 계산
        self.station_force_done = False   # 거리(STATION_FORCE_SEARCH_CM)로 강제 STATION 찾기를 이미 했는지 (한 번만)
        self.narrow_ignored_at = 0.0      # 범위 전 좁아짐 무시 로그 마지막 시각 (로그 도배 방지)
        self.yolo_ignored_at = 0.0        # 범위 전 YOLO STATION 무시 로그 마지막 시각 (로그 도배 방지)

        # STOP2 부드러운 주행 (motion.follow_gentle, 오도메트리 안 씀)
        self.gentle_corr = 0.0            # 지금 좌우 속도 차이
        self.gentle_t = None              # 마지막 계산 시각
        self.gentle_lost_since = None     # 중심선을 놓친 시각

        self.goal_seen_near = False
        self.goal_last_seen = None

        self.exit_err = None              # GOAL 구간 평활화된 오차
        self.exit_lost_since = None

        self.stop_reason = None           # 안전 정지 이유 (화면에 빨간 글씨, AUTO 켜면 지워짐)
        self.station_release = False      # STATION 정차 대기 해제 신호 (로봇팔 임무 완료 / 스페이스바)
        self.station_msg_sent = False     # STATION 도착 신호를 로봇팔에 보냈는지
        self.station_sent_at = 0.0        # 마지막으로 STATION 보낸 시각 (주기적으로 다시 보냄)

        self.confirm = TextConfirm()          # 중심선 주행 중 글씨 연속 확인
        self.narrow = RoadNarrowDetector()    # STATION 앞 도로 좁아짐 감지

    def enter(self, drive_state, msg=None):
        self.drive_state = drive_state
        self.state_start_time = time.time()
        if self.odom is not None:
            self.mark = self.odom.snapshot()   # 이 상태 시작 위치 기억 (직진 거리 / 정렬 각도 계산용)
        self.gentle_corr = 0.0                 # 부드러운 주행은 상태가 바뀔 때마다 직진에서 새로 시작
        self.gentle_t = None
        self.gentle_lost_since = None
        if msg:
            print(msg)

    def elapsed(self):
        return time.time() - self.state_start_time

    # elapsed() 와 같은 방식: "이 상태 들어온 뒤 얼마나 갔나 / 돌았나"
    def moved_cm(self):
        return 0.0 if self.odom is None else self.odom.dist - self.mark[0]

    def turned_deg(self):
        """오른쪽 회전 +, 왼쪽 회전 -"""
        return 0.0 if self.odom is None else self.odom.yaw - self.mark[1]

    def run_time(self):
        """이번 주행 소요 시간 (초). 시작 전이면 None, 완주했으면 완주 시각 기준"""
        if self.run_start_time is None:
            return None
        end = self.run_end_time if self.run_end_time is not None else time.time()
        return end - self.run_start_time

    def station_leg_cm(self):
        """STOP1 출발 후 간 거리 (cm). STOP1 을 거치지 않았으면 None"""
        if self.odom is None or self.station_leg_start is None:
            return None
        return self.odom.dist - self.station_leg_start

    def run_dist(self):
        """처음 중심선 주행을 시작한 뒤 간 거리 (cm)"""
        if self.odom is None or self.auto_start_dist is None:
            return 0.0
        return self.odom.dist - self.auto_start_dist
