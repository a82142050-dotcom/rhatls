"""상태 LED: 바퀴가 도는 명령(주행 중) = 바로 초록 / 정지 명령이 LED_RED_DELAY_SEC 이상 이어지면 빨강.
Motion.drive() 가 모터에 명령을 보낼 때마다 불러줌 (제어 루프가 매 프레임 부르므로 지연 시간이 지나면 바뀜).
색이 바뀔 때만 LED 를 갱신. LED 가 없거나 실패해도 주행은 계속."""
import threading
import time

from . import config as C

GREEN = (0, 255, 0)
RED = (255, 0, 0)


class StatusLed:
    def __init__(self, leds):
        self.leds = leds              # pinkylib LED (None 이면 아무것도 안 함)
        self.color = None
        self.stopped_since = None
        self._lock = threading.Lock()  # 제어 루프 / 웹 명령 스레드에서 같이 부름
        self._show(RED)               # 시작은 정지 = 빨강

    def set_moving(self, moving):
        if self.leds is None:
            return
        if moving:
            self.stopped_since = None
            self._show(GREEN)
            return
        now = time.time()
        if self.stopped_since is None:
            self.stopped_since = now
        if now - self.stopped_since >= C.LED_RED_DELAY_SEC:
            self._show(RED)

    def _show(self, color):
        if self.leds is None or color == self.color:
            return
        with self._lock:
            if color == self.color:
                return
            try:
                self.leds.fill(color)
                if hasattr(self.leds, "show"):
                    self.leds.show()
                self.color = color
            except Exception as e:
                print(f"[LED] 표시 실패 -> LED 없이 진행: {e}")
                self.leds = None
