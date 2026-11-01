"""핑키 LCD 에 글자 크게 띄우기 (로봇팔에서 받은 과실 인식 개수 등).
한 번 띄운 글자는 다음 메시지가 올 때까지 그대로 유지 (연결이 끊겨도 지우지 않음)."""
from PIL import Image, ImageDraw, ImageFont

from . import config as C


def _font(size):
    try:
        return ImageFont.truetype(C.LCD_FONT, size)
    except OSError:
        return ImageFont.load_default()


def _hint(e):
    """초기화 실패 이유 -> 해결 방법"""
    msg = str(e)
    if isinstance(e, ImportError):
        return "pinky_lcd 모듈을 못 찾음 -> LCD_recieve.py 와 같은 폴더의 pinky_lcd.py 를 ~/pinkyup/ksam 에 복사"
    if "busy" in msg.lower():
        return "다른 프로그램이 LCD 사용 중 -> LCD_recieve.py / 예전 main.py 를 끄고 다시 실행"
    if "different mode" in msg:
        return "pinkylib 과 pinky_lcd 의 GPIO 모드 충돌 -> 이 에러 문구 그대로 알려주기"
    return "이 에러 문구 그대로 알려주기"


class Screen:
    """핑키 LCD. 초기화에 실패해도 주행은 계속하고, 글자를 띄울 때 다시 시도"""

    def __init__(self):
        self.lcd = None
        self.error = None
        self._open()

    def _open(self):
        try:
            from pinky_lcd import LCD     # LCD_recieve.py 와 같은 방식
            self.lcd = LCD()
            self.error = None
            print("[LCD] 준비 완료")
        except Exception as e:
            self.lcd = None
            self.error = f"{type(e).__name__}: {e}"
            print(f"[LCD] 초기화 실패: {self.error}\n      -> {_hint(e)}")

    def show_text(self, text):
        """검은 화면 가운데에 흰 글자. 화면보다 길면 글자 크기를 줄여서 맞춤"""
        if self.lcd is None:
            self._open()                  # 시작할 때 실패했으면 다시 시도
        if self.lcd is None:
            print(f"[LCD] 표시 못 함: {text}")
            return
        try:
            img = Image.new("RGB", (C.LCD_WIDTH, C.LCD_HEIGHT), color=(0, 0, 0))
            draw = ImageDraw.Draw(img)

            size = C.LCD_FONT_SIZE
            while True:
                font = _font(size)
                x1, y1, x2, y2 = draw.textbbox((0, 0), text, font=font)
                if x2 - x1 <= C.LCD_WIDTH - 10 or size <= 20:
                    break
                size -= 10

            x = (C.LCD_WIDTH - (x2 - x1)) // 2 - x1
            y = (C.LCD_HEIGHT - (y2 - y1)) // 2 - y1
            draw.text((x, y), text, font=font, fill=(255, 255, 255), align="center")
            self.lcd.img_show(img)
            print(f"[LCD] 표시: {text}")
        except Exception as e:
            print(f"[LCD] 표시 실패: {type(e).__name__}: {e}")
