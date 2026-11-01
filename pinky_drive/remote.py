"""노트북(로봇팔)과 TCP 소켓 통신 (핑키봇 = 클라이언트).

받는 메시지:
  START 가 들어 있음    -> 자율주행 시작 (p 키와 같음)
  COMPLETE 가 들어 있음 -> 로봇팔 임무 완료 -> STATION 출발 (스페이스바와 같음)
  그 외                -> 받는 즉시 LCD 에 글자 표시 (과실 인식 개수 "A-1 / B-1" 등, 여러 줄이면 여러 줄로)
                          START_OK / COMPLETE 는 LCD 에 안 띄움 -> 개수 화면이 끝까지 유지됨
  한 번에 여러 메시지가 줄바꿈(\\n)으로 붙어 오면 줄마다 따로 처리 (줄바꿈이 없으면 통째로 한 메시지)
보내는 메시지 (끝에 줄바꿈):
  STATION              -> STATION 에 도착해서 정차함. 로봇팔 완료 신호가 올 때까지 주기적으로 다시 보냄
(메시지 글자는 config.py 의 MSG_* 에서 바꿀 수 있음)
"""
import socket
import threading
import time

from . import config as C


class LaptopLink:
    def __init__(self, robot, screen):
        self.robot = robot
        self.screen = screen   # lcd.Screen
        self._sock = None
        self._send_lock = threading.Lock()
        self.arm_done = False    # 로봇팔 COMPLETE 받음 -> 로봇팔 프로그램이 끝나므로 끊겨도 다시 접속 안 함 (RESET 하면 해제)

    def run(self):
        """접속 -> 신호 수신 -> 끊기면 다시 접속 (로봇 종료까지 반복). 스레드로 실행"""
        s = self.robot.s
        while not self.robot.stop_event.is_set():
            if self.arm_done:
                # 로봇팔 임무 완료 후 연결 종료 -> 조용히 대기 (접속 시도 로그 반복 안 함)
                time.sleep(0.5)
                continue
            sock = self._connect()
            if sock is None:
                return
            self._sock = sock

            # STATION 에서 기다리는 중에 (다시) 연결됐으면 도착 신호를 다시 보냄
            if s.drive_state == "STATION_WAIT" and self.send(C.MSG_STATION_ARRIVED):
                s.station_msg_sent = True

            try:
                self._receive(sock)
            finally:
                self._sock = None
                sock.close()
            s.link_status = "disconnected"
            if self.arm_done:
                s.link_status = "arm done"
                print("[핑키봇] 로봇팔 임무 완료 + 연결 종료 -> 더 이상 접속 시도 안 함 (웹 RESET 누르면 다시 접속)")
            else:
                print("[핑키봇] 노트북 연결 끊김 -> 다시 접속 시도")

    def send(self, msg, quiet=False):
        """로봇 -> 노트북. 연결 안 돼 있거나 실패하면 False (quiet=True 면 로그 안 찍음)
        끝에 줄바꿈을 붙임: 여러 번 보낸 게 붙어 와도 로봇팔이 단어로 나눠서 'STATION' 을 인식하게"""
        sock = self._sock
        if sock is None:
            if not quiet:
                print(f"[핑키봇] 노트북 연결 안 됨 -> '{msg}' 못 보냄")
            return False
        try:
            with self._send_lock:
                sock.sendall((msg + "\n").encode("utf-8"))
        except OSError as e:
            if not quiet:
                print(f"[통신 에러] '{msg}' 보내기 실패: {e}")
            return False
        if not quiet:
            print(f"[핑키봇 -> 노트북] {msg}")
        return True

    def _connect(self):
        """노트북 서버가 켜질 때까지 계속 접속 시도"""
        self.robot.s.link_status = "connecting"
        print(f"[핑키봇] 노트북({C.LAPTOP_IP}:{C.LAPTOP_PORT})에 접속 시도 중...")

        while not self.robot.stop_event.is_set():
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(C.LINK_CONNECT_TIMEOUT)
            try:
                sock.connect((C.LAPTOP_IP, C.LAPTOP_PORT))
            except OSError:
                sock.close()
                time.sleep(C.LINK_RETRY_SEC)
                continue

            sock.settimeout(None)
            self.robot.s.link_status = "connected"
            print("[핑키봇] 노트북 연결 성공! 신호를 기다립니다...")
            return sock
        return None

    def _receive(self, sock):
        """주행 내내 백그라운드에서 신호를 계속 받음. 연결이 끊기면 return"""
        while True:
            try:
                raw = sock.recv(1024)
            except OSError as e:
                print(f"[통신 에러] {e}")
                return
            if not raw:            # 빈 바이트 = 상대가 연결을 닫음 (줄바꿈만 온 경우와 구분)
                return

            lcd_lines = []
            for data in raw.decode("utf-8", errors="ignore").splitlines():
                data = data.strip()
                if not data:
                    continue
                print(f"[수신] {data}")
                if C.MSG_START in data:
                    print("[핑키봇] 노트북으로부터 주행 시작 명령을 받았습니다. 주행을 시작합니다!")
                    self.robot.start_auto("SOCKET")
                elif C.MSG_ARM_DONE in data:
                    print(f"[핑키봇] 로봇팔 임무 완료 수신: {data}")
                    self.arm_done = True
                    self.robot.release_station("ARM")
                else:
                    lcd_lines.append(data)

            # 받는 즉시 표시 (로봇팔이 "A-1\nB-1" 처럼 두 줄로 보내면 두 줄로), 연결이 끊겨도 LCD 는 그대로 유지
            if lcd_lines:
                self.screen.show_text("\n".join(lcd_lines))
