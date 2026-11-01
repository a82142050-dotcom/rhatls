"""Pinky Pro GOAL 완주 시 재생할 악보 기반 단선율 멜로디.

출처: 사용자가 제공한 영상(화면 녹화 중 2026-11-01 043931.mp4)의
'La Victoria - She's so BAD' 화면 악보 67~73마디.
보컬의 X자 음표는 음높이 없는 리듬 음표여서 해당 구간은 보이는
Obbligato(보조 선율)를 대신 사용했습니다. 리듬은 부저 연주에 맞게 근사했습니다.
"""
import time
from pinkylib import Buzzer

BPM = 120  # 빠르면 올리고 느리면 내리세요
DUTY = 45  # 0~99: 부저 출력 크기

# 악보는 6개 플랫 (Gb 장조 / Eb 단조)로 보임
# Bb=시♭, Cb=도♭, Eb=미♭
FREQ = {
    "Bb3": 233,
    "Bb4": 466,
    "Cb5": 494,
    "Eb5": 622,
}

# (음이름, 박자 수): None은 쉼표 / 1=4분음표 / 0.5=8분음표
# 67~69마디: 보컬 X자 음표 대신 아래쪽 보조 선율
MELODY = [
    # 67마디: She's so
    (None, 0.25), ("Bb4", 0.75), ("Cb5", 0.5), ("Eb5", 0.5),
    ("Bb4", 1.0), ("Bb3", 1.0),
    # 68마디: BAD
    ("Bb4", 1.0), ("Cb5", 0.5), ("Eb5", 0.5),
    ("Bb4", 1.0), (None, 1.0),
    # 69마디: She's so
    (None, 0.25), ("Bb4", 0.75), ("Cb5", 0.5), ("Eb5", 0.5),
    ("Bb4", 1.0), ("Bb3", 1.0),
    # 70마디: BAD every day (보컬 악보)
    ("Bb4", 1.0), ("Cb5", 0.5), ("Eb5", 0.5),
    ("Bb4", 1.0), (None, 1.0),
    # 71마디: Way she's moving up the tempo
    (None, 0.25), ("Bb4", 0.75), ("Bb4", 0.25),
    ("Cb5", 0.5), ("Eb5", 0.5), ("Bb4", 0.5),
    ("Bb4", 0.5), ("Cb5", 0.5), ("Cb5", 0.5),
    # 72마디: Switching up the pace
    ("Bb4", 0.75), ("Bb4", 0.25), ("Cb5", 0.5),
    ("Eb5", 0.5), ("Bb4", 1.0), (None, 1.0),
    # 73마디: Love the way she's going loco
    (None, 0.25), ("Bb4", 0.75), ("Bb4", 0.25),
    ("Cb5", 0.5), ("Eb5", 0.5), ("Bb4", 0.5),
    ("Bb4", 0.5), ("Cb5", 0.5), ("Cb5", 0.5),
    # 끝맺음 (원곡 전체가 아니라 영상에 보이는 구절만 연주)
    ("Bb4", 1.0),
]


def play_goal_song():
    """부저로 한 번 재생하고 GPIO 자원을 반드시 반환."""
    buzzer = None
    try:
        buzzer = Buzzer()
        buzzer.buzzer_start()
        beat_sec = 60.0 / BPM

        for note, beats in MELODY:
            seconds = beat_sec * beats
            if note is None:
                buzzer.set_buzzer_duty(0)
                time.sleep(seconds)
                continue

            buzzer.set_buzzer_freq(FREQ[note])
            buzzer.set_buzzer_duty(DUTY)
            time.sleep(seconds * 0.90)
            buzzer.set_buzzer_duty(0)
            time.sleep(seconds * 0.10)

    except Exception as e:
        print(f"[GOAL SONG] 부저 재생 오류: {e}")
    finally:
        if buzzer is not None:
            try:
                buzzer.set_buzzer_duty(0)
            finally:
                buzzer.close()


if __name__ == "__main__":
    # 로봇의 라즈베리파이에서만 실행 가능 (pinkylib 필요)
    play_goal_song()
