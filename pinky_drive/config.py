"""튜닝 파라미터 모음. 현장에서 값 조정은 이 파일만 보면 된다.

코스 순서: STOP1 -> STATION -> STOP2 -> (직진 + 우회전) -> GOAL -> DONE
"""
import os

import numpy as np


# ============================================================
# Network
# ============================================================
PORT = 5000                      # 웹 화면 포트

# 노트북 TCP 서버 (핑키봇이 클라이언트로 접속)
#   run_all.py 로 실행하면 SSH 터널을 쓰므로 PINKY_LAPTOP_IP=127.0.0.1 이 자동으로 넘어옴 (IP 설정 필요 없음)
#   main.py 를 로봇에서 직접 실행할 때만 아래 기본값(노트북 IP)을 사용
LAPTOP_IP = os.environ.get("PINKY_LAPTOP_IP", "192.168.4.12")
LAPTOP_PORT = 6000
LINK_RETRY_SEC = 1.0             # 노트북 서버가 안 켜져 있거나 끊기면 이 간격으로 다시 접속 시도
LINK_CONNECT_TIMEOUT = 3.0       # 접속 시도 1회 최대 대기 시간

# ★ 노트북(로봇팔)과 주고받는 메시지 - 로봇팔 코드와 글자를 맞출 것
MSG_START = "START"              # 받음: 이 글자가 들어 있으면 자율주행 시작 (START_AUTONAV 도 OK)
MSG_STATION_ARRIVED = "STATION"  # 보냄: STATION 에 도착해서 정차했을 때
MSG_ARM_DONE = "COMPLETE"        # 받음: 이 글자가 들어 있으면 로봇팔 임무 완료 -> STATION 출발
                                 #       (예: 로봇팔이 "MISSION_COMPLETE" 를 보내면 됨)
STATION_RESEND_SEC = 2.0         # STATION 에서 기다리는 동안 이 간격으로 STATION 을 다시 보냄
                                 #   (로봇팔은 스캔 중에 받은 STATION 을 무시함 -> 스캔 끝난 뒤 다시 받아야 수확 시작)


# ============================================================
# LCD (노트북에서 받은 글자 표시)
# ============================================================
LCD_WIDTH = 320                  # 핑키 LCD 해상도 (LCD_recieve.py 와 같음)
LCD_HEIGHT = 240
LCD_FONT = "DejaVuSans.ttf"
LCD_FONT_SIZE = 45               # 글자가 화면보다 길면 자동으로 줄임

# 상태 LED: 주행 중 초록 / 정지 빨강
LED_RED_DELAY_SEC = 0.5          # 정지가 이 시간 이상 이어질 때만 빨강 (글자 정렬 중 잠깐 멈춤에 깜빡이지 않게)


# ============================================================
# ★★ 전체 주행 속도 배율 (1.0 = 예전 속도, 1.2 = 20% 빠르게)
#   모든 주행 속도 + 조향 세기(KP 등)를 같은 비율로 바꿈 -> 같은 경로를 더 빨리 감
#   거리(cm) / 각도(deg) 목표는 실제 위치라서 속도와 상관없이 그대로 (오도메트리가 알아서 맞춤)
#   수동 조작(MANUAL_SPEED) 과 바퀴 최소 속도(MIN_MOVE / MIN_TURN_SPEED) 는 그대로
# ============================================================
SPEED_SCALE = 2.09   # [1.2 -> 1.44 -> 1.58 -> 1.74 -> 2.09 (20% 더)]  ※ 제자리 회전 4개는 아래에 고정값 (배율 무관)


# ★ STATION 찾는 구간 (STOP1 끝 ~ STATION 정차) 만 이 배율로 (SPEED_SCALE 보다 크면 무시)
#   빠르면 좁아짐 감지가 켜지기 전에 지나가거나 STATION 글씨를 놓치고 가운데 구조물에 부딪힘
#   주행 속도 + 조향이 같이 이 배율 기준으로 돌아감. 제자리 회전(고정 속도)은 그대로
STATION_SPEED_SCALE = 1.74
STATION_SLOW_STAGES = {"STATION"}
STATION_SLOW_EXCLUDE_STATES = {"STATION_SEARCH_TURN", "TEXT_SEEK_TURN"}   # 제자리 회전 상태는 고정 속도 유지


def _s(speed):
    """기준 속도 x SPEED_SCALE (모터 명령은 정수)"""
    return round(speed * SPEED_SCALE)


# ============================================================
# 기본 주행   ([ ] 안은 SPEED_SCALE 1.0 기준값)
# ============================================================
BASE_SPEED = _s(20)
KP = 0.12 * SPEED_SCALE
MAX_SPEED = _s(32)

CURVE_SLOWDOWN = 0.05 * SPEED_SCALE   # 중심선 오차가 커질수록 기본 속도 감소
MIN_CURVE_SPEED = _s(12)    # 커브 최소 속도
INNER_MIN_SPEED = _s(-10)   # 안쪽 바퀴 역회전 허용 속도

SEARCH_SPEED = _s(16)
SEARCH_INNER_SPEED = _s(-6)  # 중심선 이탈 시 안쪽 바퀴 역회전 탐색

TEXT_FOLLOW_SPEED = _s(16)   # 글씨 따라가기 + 오도메트리 직진 속도
TEXT_FOLLOW_KP = 0.10 * SPEED_SCALE
TEXT_FOLLOW_MAX_CORR = _s(10)

TURN_SPEED = 22              # 처음 중심선 주행 직후 STOP1 찾는 제자리 회전 속도 [고정: SPEED_SCALE 무관, YOLO 인식용]

MANUAL_SPEED = 24
MANUAL_PULSE = 0.30
JPEG_QUALITY = 55

# ★ 프레임 속도 (라즈베리파이 제어 루프). 화면 오른쪽 위 FPS 로 확인
LOOP_MAX_FPS = 30      # 제어 루프 최대 속도 (카메라 30fps). 예전: 매 프레임 끝에 0.03초 고정으로 쉼
WEB_VIEW_EVERY = 3     # 웹 화면(그림 + JPEG 압축)은 이 프레임마다 한 번만 만듦. 주행 제어는 매 프레임 그대로

# ★ 시작 후 이 거리만큼은 YOLO 없이 중심선 추종 (오도메트리)
#   [예전 25초 = BASE_SPEED 20 x SPEED_TO_CM_PER_SEC 0.43 x 25초 ≈ 215cm]
#   STOP 앞까지 못 가고 멈추면 올리고, STOP 을 지나쳐서 멈추면 내릴 것
CENTERLINE_RUN_CM = 212.0   

# ★ [안전] 처음 중심선 주행 직후 제자리 회전 탐색(SEARCH_TEXT) 최대 시간
#   기준 속도 18 ≈ 초당 89도 -> 8초면 약 두 바퀴 (속도가 빨라지면 같은 두 바퀴가 되도록 시간도 줄임)
#   못 찾으면 정지 + AUTO 해제 -> 로봇 위치 잡고 AUTO 누르면 중심선 주행(YOLO 켜진 상태)부터 재개
SEARCH_TEXT_TIMEOUT_SEC = 4.6   # [고정: 회전 속도 22 기준 약 1.4바퀴]


# ============================================================
# ★★ 오도메트리: 거리(cm) / 각도(deg)로 끝나는 동작
#   거리 = 처음 중심선 주행 / 좁아짐 후 직진 / 글씨 후 직진 / STOP2 후 직진 / GOAL 후 직진
#   각도 = 글자 정렬(TEXT_SEEK) / STATION 찾기 회전 / STOP2 후 마지막 우회전
#   초 단위로 남은 것 = 3초 정지, 못 찾을 때 포기하는 시간(타임아웃)
# ============================================================
# 엔코더 1틱당 이동 거리. 측정: 1m(100cm) 직진시키고  100 / (평균 틱 수)
CM_PER_TICK = 0.05

# 양쪽 바퀴 사이 거리(cm). 측정: 제자리에서 360도 x 5바퀴 돌려 보고
#   덜 돌면 값을 줄이고, 더 돌면 값을 늘릴 것  (IMU 쓰면 회전에는 안 쓰임)
WHEEL_BASE_CM = 10.0

# [센서 없을 때 추정용] 속도 명령 1 = 초당 몇 cm 인지.
#   측정: 속도 20 으로 3초 직진 -> 간 거리(cm) / 60
#   지금 값은 예전 "18 속도로 0.9초에 80도"에 맞춰 환산한 시작값
#   ※ 속도 "1" 당 값이라 SPEED_SCALE 을 바꿔도 그대로 둠 (빨라지면 추정 거리도 같은 비율로 빨리 늘어남)
#     빨라진 뒤 실제 간 거리가 화면 ODOM 과 다르면(미끄러짐 등) 이 값만 다시 측정
SPEED_TO_CM_PER_SEC = 0.43

# IMU yaw 부호: 오른쪽으로 돌 때 값이 줄어드는 IMU 면 -1
IMU_YAW_SIGN = 1

# 목표 근처에서 감속 (덜 밀려나게)
SLOW_ZONE_CM = 5.0           # 남은 거리가 이보다 작으면 감속
SLOW_ZONE_DEG = 20.0         # 남은 각도가 이보다 작으면 감속
SLOW_ZONE_RATIO = 0.6        # 감속할 때 속도 비율
MIN_MOVE_SPEED = 10          # 이보다 낮으면 바퀴가 안 굴러가는 로봇이면 올릴 것
MIN_TURN_SPEED = 12

# 직진 중 방향 보정 (엔코더/IMU 있을 때만 효과). 1도 틀어질 때 좌우 속도 차이
HEADING_KP = 0.5 * SPEED_SCALE

# 안전장치: (예상 시간 x 이 배수 + 1초) 가 지나면 목표에 못 가도 멈춤
MOVE_TIMEOUT_FACTOR = 2.0


# ============================================================
# 중심선 검출 (ROI / HSV)
# ============================================================
ROI_START_RATIO = 0.40

# GOAL 구간에서만 사용하는 오른쪽 ROI 제외 설정
GOAL_RIGHT_MASK_DELAY_SEC = 2.5  # CENTERLINE 진입 후 2.5초
GOAL_RIGHT_MASK_RATIO = 0.10     # 화면 오른쪽 10% 제외


LOWER_WHITE = np.array([0, 0, 175], dtype=np.uint8)
UPPER_WHITE = np.array([180, 75, 255], dtype=np.uint8)

# ★ [조명 대응] 카메라 노출 / 화이트밸런스 고정 (main.py 에서 카메라 켠 직후)
#   켤 때 CAMERA_SETTLE_SEC 동안 카메라가 그 장소 조명에 자동으로 맞춘 뒤 그 값으로 잠금 -> 주행 중 화면 밝기 / 색이 안 흔들림
#   (자동 노출은 화면에 검은 게 많으면 밝게, 흰 게 많으면 어둡게 계속 바꿔서 같은 도로인데 밝기가 달라짐)
#   ※ 출발 위치에서 도로를 보고 있는 상태로 켤 것. 장소 / 조명이 바뀌면 프로그램을 다시 켤 것 (RESET 으로는 다시 안 맞춤)
CAMERA_LOCK_EXPOSURE = True        # False = 예전처럼 계속 자동
CAMERA_SETTLE_SEC = 2.0            # 잠그기 전에 자동으로 맞추는 시간
CAMERA_BRIGHTNESS_BOOST = 1.0      # 잠글 때 밝기 배율 (1.0 = 자동으로 맞춘 그대로) [1.2 / 1.1 시험 -> 1.0: STATION 회전교차로 말고는 황토 라인 밖까지 하얗게 잡힘]
                                   #   그림자 진 흰 도로가 회색으로 끊기면 1.2~1.3, 화면이 하얗게 날아가서 주황/초록이 바래면 0.8

# ★ [조명 대응] 고정 HSV(위 V>=175) + 자동 이진화(CLAHE + Otsu, 화면마다 도로/바닥 기준 자동) 를 합쳐서 흰 도로를 찾는 구간
#   어둡거나 그림자 진 도로도 흰색으로 잡힘. 밝을 때는 고정 HSV 결과 그대로. 자동 이진화 설정은 아래 EXIT_V_MIN / EXIT_S_MAX 공용
#   바닥 반사가 도로로 잡히면 EXIT_V_MIN 을 올리거나, 구간을 빼서 예전처럼 (예전: {"GOAL"} 만)
ADAPTIVE_WHITE_STAGES = {"STOP1", "STATION", "STOP2", "GOAL"}

MIN_AREA = 500
STEP = 15
MIN_WIDTH = 25
MAX_JUMP = 60
MAX_POINTS = 8
INTERNAL_GAP_RATIO = 0.30


# ============================================================
# 코스 정의
# ============================================================
# route_stage 별로 찾아야 하는 글씨
STAGE_TARGET = {"STOP1": "STOP", "STATION": "STATION", "STOP2": "STOP", "GOAL": "GOAL"}
# STOP_3SEC 이 끝난 뒤 다음 route_stage (STOP2 는 FINAL_FORWARD 로 따로 처리)
NEXT_STAGE = {"STOP1": "STATION", "STATION": "STOP2"}


# ============================================================
# ★ [조정용] STOP2 구간: 우측 오프셋 중심선 (화면에 주황색 선)
#   0.0 = 도로 정중앙, 1.0 = 도로 오른쪽 가장자리, 음수 = 왼쪽
# ============================================================
CENTERLINE_RIGHT_OFFSET_RATIO = 0.27   # [0.15 -> 0.27 -> 0.47 -> 0.27] 더 오른쪽으로 가려면 올리고, 오른쪽 선을 밟으면 내릴 것
RIGHT_OFFSET_STAGES = {"STOP2"}

# STOP2 구간: 화살표를 흰 도로로 채워서 중심선이 흔들리지 않게 함
ARROW_FILL_STAGES = {"STOP2"}
ARROW_GAP_RATIO = 0.80
ARROW_CLOSE_KERNEL = 15


# ============================================================
# ★★ [조정용] STOP2 인식하러 갈 때 부드럽게 주행 ("고개 확 돌리기" 줄이기, 오도메트리 안 씀)
#   예전: KP 0.12 + 안쪽 바퀴 역회전(-10) 허용 -> 오차 150px 이면 바퀴 (30, -5) 로 확 꺾임
#         중심선을 놓치면 마지막 오차 쪽으로 (16, -6) 제자리 회전
#   지금: 조향 절반 + 좌우 차이 상한 + 안쪽 바퀴 역회전 없음 + 조향이 천천히 바뀜 + 놓치면 잠깐 직진
# ============================================================
GENTLE_STAGES = {"STOP2"}       # 적용 구간. set() 로 비우면 예전 방식 그대로
GENTLE_SPEED = _s(16)           # 기본 속도
GENTLE_TEXT_SPEED = _s(12)      # YOLO에 STOP이 보이기 시작하면 이 속도로 (확인 프레임을 더 확보)
GENTLE_KP = 0.06 * SPEED_SCALE  # 조향 세기 (기본 KP 의 절반)
GENTLE_MAX_CORR = _s(6)         # ★ 좌우 바퀴 속도 차이 최대 (기본 속도보다 작아서 안쪽 바퀴 역회전 없음)
                                #   아직도 확 돌면 4, 커브를 못 따라가고 밖으로 나가면 8
GENTLE_CORR_RATE = 10 * SPEED_SCALE   # 좌우 속도 차이가 1초에 최대 이만큼만 바뀜 (갑자기 확 안 꺾임)
GENTLE_LOST_STRAIGHT_SEC = 1.0  # 중심선을 놓쳐도 이 시간 동안은 제자리 회전 대신 천천히 직진
GENTLE_LOST_SPEED = _s(12)      #   그때 속도. 그래도 못 찾으면 바깥 바퀴만 이 속도로 천천히 좌우를 살펴봄
GENTLE_SWEEP_SEC = 1.0          #   마지막 오차 쪽으로 이 시간(약 30도) 돌고, 없으면 반대쪽으로 2배 ...
                                #   (예전: 찾을 때까지 한쪽으로 계속 돌아서 한 바퀴 넘게 돌기도 함)


# ============================================================
# GOAL 구간(STOP2 이후 우회전 -> GOAL) 주행 안정화
EXIT_STAGES = {"GOAL"}
EXIT_V_MIN = 120
EXIT_S_MAX = 90
EXIT_ERR_SMOOTH = 0.6

# GOAL 중심선 추종 속도 및 조향 제한
EXIT_FOLLOW_SPEED = _s(14)       # 추가: 속도 감소
EXIT_MAX_CORR = _s(8)           # 조향 상한 (기존 수정값 6 → 8)

# 중심선 유실 시
EXIT_LOST_STRAIGHT_SEC = 0.25   # 기존 0.6 → 0.25
EXIT_LOST_SPEED = _s(14)

# ============================================================
# YOLO
# ============================================================
MODEL_PATH = "best_ncnn_model"
YOLO_CONF = 0.45
YOLO_IMGSZ = 320

# ★ 노트북에서 YOLO 추론 (노트북 yolo_server.py 에 ROI 이미지를 보내고 박스만 받음)
#   연결 안 되거나 답이 늦으면 그 프레임은 라즈베리파이에서 직접 추론 (best_ncnn_model 은 대비용으로 항상 로드)
#   run_all.py 로 실행하면 SSH 터널로 127.0.0.1 사용 (IP 설정 필요 없음)
YOLO_REMOTE = True                     # False 면 예전처럼 라즈베리파이에서만 추론
YOLO_SERVER_IP = os.environ.get("PINKY_YOLO_IP", LAPTOP_IP)
YOLO_SERVER_PORT = 7000
YOLO_REMOTE_JPEG_QUALITY = 80          # 보내는 이미지 화질 (낮을수록 빠르지만 인식이 떨어질 수 있음)
YOLO_REMOTE_TIMEOUT = 0.5              # 이 시간 안에 답이 없으면 라즈베리파이에서 추론하고 다시 연결
YOLO_REMOTE_RETRY_SEC = 3.0            # 노트북 연결이 끊겼을 때 다시 시도하는 간격
TEXT_CLASSES = {"STOP", "STATION", "GOAL"}

# ★ [조정용] 화살표 인식 3중 차단
#  1) 모델에 STOP/STATION/GOAL 클래스만 요청
#  2) 이름에 아래 단어가 있으면 버림
#  3) 박스 가로/세로 비율이 아래 값보다 작으면(글씨가 아닌 모양) 버림 -> 화면에 회색 REJECT
ARROW_BLOCK_WORDS = ("ARROW", "LEFT", "RIGHT", "STRAIGHT", "TURN", "화살")
TEXT_MIN_ASPECT = {"STOP": 1.5, "STATION": 2.0, "GOAL": 1.3}

# ★ [조정용] 구간별 최소 신뢰도 (없는 구간은 YOLO_CONF)
#   STOP2: 화살표가 있어서 더 엄격
#   GOAL : 모델 신뢰도가 낮게 나와서 낮춤 (엉뚱한 곳에 GOAL이 잡히면 0.30~0.35로 올릴 것)
STAGE_MIN_CONF = {"STATION": 0.35, "STOP2": 0.60, "GOAL": 0.25}


# ============================================================
# 글씨 확인 (중심선 주행 중)
# ============================================================
# 글씨가 이 프레임 수만큼 연속으로 보여야 이벤트 실행 (화살표 순간 오인식 무시)
EVENT2_CONFIRM_FRAMES = 3
EVENT2_CONFIRM_FRAMES_BY_TYPE = {"GOAL": 2, "STATION": 2}   # GOAL/STATION은 2프레임만 확인

# 화살표 없는 구간에서는 한 프레임 놓쳐도 카운트를 0으로 리셋하지 않고 1만 깎음
# 여기 있는 구간은 화살표 때문에 놓치면 바로 0으로 리셋
EVENT2_STRICT_STAGES = {"STOP2"}

# ★ [조정용] 확인하는 동안 "전체 보임"이 이 프레임 이상이었으면 멈춰서 다시 확인하지 않고 바로 접근
#   (예전: 확인 3프레임 -> 멈춤 -> 전체 보임 3프레임 더 확인 = 라즈베리파이에서 약 3초)
FAST_FULL_FRAMES = 2


# ============================================================
# ★ [조정용] 글자 전체 확인 / 정렬 회전 (SEARCH_TEXT / TEXT_SEEK_FULL 공통)
# ============================================================
FULL_MARGIN = 20           # 박스 좌우가 화면 끝에서 이 픽셀 이상 떨어져야 "전체 보임"
                           # (다시 STATIO에서 출발하면 30~35로 올릴 것)
FULL_STABLE_FRAMES = 2     # 전체 보임이 연속 몇 프레임 유지돼야 출발할지

# ★ 정렬(TEXT_SEEK_FULL): 예전 "0.12초 돌고 0.15초 멈춤" 대신 오도메트리로 "몇 도 돌고 -> 멈춰서 보기" 반복
#   (예전 방식은 YOLO 때문에 루프가 느리면 회전 명령이 다음 프레임까지 유지돼서 한 번에 20도 넘게 돌았음)
SEEK_STEP_DEG = 8          # 한 번에 도는 각도 (그래도 많이 돌면 5)
SEEK_STEP_SPEED = 21       # 그때 회전 속도 [고정: SPEED_SCALE 무관]
SEEK_SETTLE_FRAMES = 1     # 멈춘 직후 이 프레임 수는 흔들려서 판단에 안 씀
SEEK_MISS_FRAMES = 4       # 글씨가 이 프레임 연속 안 보이면 중심선 추종으로 복귀 (예전 SEEK_LOST_TIMEOUT)
SEEK_MAX_DEG = 40          # 정렬 시작 방향에서 이 각도 넘게는 안 돌림 -> 그때는 보이는 대로 접근


# ============================================================
# ★★★ [조정용] STATION 앞 "도로 좁아짐" 감지 -> 멈추고 오른쪽으로 돌려 STATION 찾기 ★★★
# STOP1 이후 STATION 찾는 구간에서만 동작
#   흰 도로 폭이 평소보다 갑자기 좁아지고 + 중심선이 옆으로 쏠리면
#   -> 조금 더 직진 -> 멈춤 -> 오른쪽으로 "몇 도 돌고 -> 멈춰서 보기" 반복하며 STATION 박스 찾기
#   -> 찾으면 정렬(TEXT_SEEK_FULL)
# (YOLO가 주행 중에 먼저 STATION을 잡으면 그쪽이 우선)
#
# 화면 STATION 구간에 "ROAD W / BASE / NARROW" 가 표시되니 그 숫자를 보고 조정할 것
# ============================================================
NARROW_RATIO = 0.60          # 도로 폭이 평소(BASE)의 60% 아래로 줄면 "좁아짐"
                             #   너무 자주 멈추면 0.50, 안 멈추면 0.70
NARROW_SIDE_ERR = 35         # 중심선 오차가 이 픽셀 이상 옆으로 쏠려야 함
                             #   0 으로 하면 폭만 보고 판단
NARROW_CONFIRM_FRAMES = 2    # 좁아짐이 연속 몇 프레임이어야 멈출지
NARROW_FORWARD_CM = 6.0      # ★ 좁아짐 감지 후 이 거리만큼 더 직진하고 멈춤 (0 이면 바로 멈춤) [예전 0.85초]
NARROW_FORWARD_SPEED = _s(16)   #   그때 직진 속도
NARROW_ARM_SEC = 2.0         # STOP1 끝나고 이 시간 동안은 감지 안 함 (평소 폭 학습 시간) [2.0 -> 4.0 -> 2.0: 시작 위치는 NARROW_MIN_CM(거리)이 정함]
NARROW_BASE_FRAMES = 15      # 평소 폭(BASE)을 최근 몇 프레임의 중간값으로 잡을지
NARROW_COOLDOWN_SEC = 3.0    # 못 찾고 복귀한 뒤 이 시간 동안은 다시 감지 안 함

# ★★ [조명 대응] STOP1 출발 후 간 거리(오도메트리 cm)로 STATION 찾는 범위를 제한
#   STOP1 출발 ~ NARROW_MIN_CM     : 도로 좁아짐이 감지돼도 무시 (조명/그림자 때문에 일찍 회전하는 것 방지)
#   NARROW_MIN_CM ~                : 좁아짐 감지되면 지금처럼 NARROW_FORWARD_CM 더 가서 오른쪽으로 찾기
#   STATION_FORCE_SEARCH_CM 도착   : 좁아짐을 못 잡았어도 그 자리에서 멈추고 오른쪽으로 찾기 (한 번만)
#   정하는 법: 좁아짐이 제대로 걸렸을 때 로그 "road narrowed ... STOP1 +Ncm" (화면 STATION 구간에도 표시) 의 N 기준으로
#             NARROW_MIN_CM = N - 20, STATION_FORCE_SEARCH_CM = N + 15 정도. 여러 번 돌려서 N 이 흔들리는 만큼 범위를 넓힐 것
#   0 이면 끔 (실측 전이라 지금은 둘 다 0 = 예전과 똑같이 동작)
NARROW_MIN_CM = 60.0                # [10-21 실측 1회: STOP1 +60cm 에서 좁아짐 -> +66cm 에서 찾기 시작해 STATION 찾음]
STATION_FORCE_SEARCH_CM = 75.0      # [그 +15cm. STATION 정차는 +104cm]
# ★ STOP1 출발 후 이 거리 전에는 달리면서(중심선 주행 중) YOLO 가 STATION 을 봐도 무시 -> 좁아짐 / 강제 찾기로만 찾음
#   [10-22] YOLO 가 +59cm 에서 먼저 보고 바로 꺾어서 안쪽으로 파고듦 -> 정차 위치가 옆으로 쏠림
#   (멈춰서 오른쪽으로 돌며 찾을 때의 YOLO 는 그대로 사용). 0 이면 끔 (예전처럼 달리면서 봐도 바로 정렬)
STATION_YOLO_MIN_CM = 75.0

# ★ STATION 찾기 회전 (오도메트리): 예전 "0.12초 돌고 0.15초 멈춤" 시간 펄스 5초 대신
STATION_SEARCH_STEP_DEG = 15     # 한 번에 오른쪽으로 도는 각도
STATION_SEARCH_SPEED = 24        # 그때 회전 속도 [고정: SPEED_SCALE 무관]
STATION_SEARCH_LOOK_FRAMES = 2   # 돌고 나서 멈춰서 YOLO 로 보는 프레임 수
STATION_SEARCH_MAX_DEG = 150     # 이만큼 돌아도 못 찾으면 중심선 복귀 [예전 5초 ≈ 150도]


# ============================================================
# 글씨 접근 / 정지 (STOP / STATION 공통)
# ============================================================
TEXT_ALIGN_TRIGGER_RATIO = 0.60    # 박스 하단이 화면 60%까지 오면 DRIVE_TEXT
TEXT_BOTTOM_TRIGGER_RATIO = 0.98   # 박스 하단이 화면 바닥에 닿으면 직진 시작
FORWARD_AFTER_TEXT_CM = 20.0       # 박스 하단 도달 후 직진 거리 [예전 3초]
STOP_AFTER_TEXT_SEC = 3.0          # 그 뒤 정지 시간 (정지는 초 단위 그대로, STATION 은 로봇팔 신호 대기)


# ============================================================
# STOP2 이후 -> GOAL
# ============================================================
FINAL_STOP_FORWARD_CM = 15.0       # STOP2 후 우회전 전까지 직진 거리 [예전 3초]
# ★ [조정용] STOP2 이후 마지막 우회전 각도 (오도메트리: 이 각도만큼 돌면 멈춤) [예전 1.35초]
#   더 돌아야 하면 FINAL_TURN_DEG 를 올리고, 너무 돌면 내릴 것
#   화면 ODOM 각도는 맞는데 실제로 덜/더 돌면 WHEEL_BASE_CM 조정 (IMU 쓰면 정확)
FINAL_TURN_DEG = 150
FINAL_TURN_SPEED = 31        # [고정: SPEED_SCALE 무관]

# ============================================================
# GOAL 새 도로 탐색
# ============================================================

# 새 도로 판정
GOAL_LANE_MIN_SLOPE = 0.13
GOAL_LANE_CONFIRM_FRAMES = 3

# 정지 후 카메라 확인
GOAL_LANE_SETTLE_SEC = 0.20
GOAL_LANE_LOOK_SEC = 0.80

# 새 도로를 못 찾았을 때 추가 우회전
GOAL_LANE_STEP_DEG = 30
GOAL_LANE_MAX_STEPS = 5
GOAL_LANE_TURN_SPEED = 31

# ★ [조정용] GOAL: 박스를 따라 직진 -> 박스가 사라지면 GOAL_FINAL_FORWARD_CM 더 직진 -> 완전 정지
GOAL_NEAR_RATIO = 0.70         # 박스 하단이 화면 70% 아래까지 오면 "가까이 왔다"
GOAL_MISS_TIMEOUT = 0.40       # 가까이 온 뒤 이 시간 동안 안 보이면 "사라졌다"
GOAL_GIVEUP_SEC = 3.0          # 멀리서 놓친 채 이 시간이 지나면 그래도 마무리 직진
GOAL_FINAL_FORWARD_CM = 36.0   # 사라진 뒤 추가 직진 거리 [41 -> 38 -> 36] [48cm 에서 약 1초 빨리 멈추게 줄임: 속도 16 x 0.43 = 초당 약 7cm]
GOAL_SPEED_BOOST = 1.2     # ★ GOAL 인식 후(박스 따라 직진 + 마지막 직진)만 글씨 따라가기 속도보다 빠르게 [1.2 -> 1.44: 20% 더]
GOAL_SPEED = round(TEXT_FOLLOW_SPEED * GOAL_SPEED_BOOST)   # 거리(cm)는 그대로, 도착만 빨라짐
