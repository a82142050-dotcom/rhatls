"""Pinky Pro GOAL 완주 음악 — 제공된 악보 9~12마디의 '위 오선' 기준.
"""
import time

BPM = 130
REPEAT_COUNT = 2
DUTY = 45  # Pinky Pro 부저 duty (0~99)

FREQUENCIES = {
    "C4": 262,
    "F4": 349,
    "G4": 392,
    "Bb4": 466,
    "C5": 523,
    "Eb5": 622,
    "G5": 784,
}

# 정확히 4/4 마디당 4박. 9~12마디 총 16박.
# 각 항목: (음, 박수, 스타카토, 다음 음까지 슬러/레가토)
SCORE_BARS = [
    # 9마디 Cm  : [C5/G4/Eb4] 화음(윗음 C5) - C5 - Bb4 - C5 - Eb5 - C5
    [("C5", 1.0, False, False),
     ("C5", 1.0, False, False),
     ("Bb4", 0.5, False, False),
     ("C5", 0.5, False, False),
     ("Eb5", 0.5, False, False),
     ("C5", 0.5, False, False)],

    # 10마디 Eb9 : 첫 화음은 윗음 G4, '속-은' 사이 슬러 표기
    [("G4", 1.0, False, False),
     ("G4", 0.5, False, True),
     ("F4", 0.5, False, False),
     ("F4", 0.5, False, False),
     ("G4", 0.5, False, False),
     ("Bb4", 0.5, False, False),
     ("G4", 0.5, False, False)],

    # 11마디 Abmaj7 : 위쪽 화음의 최고음, 2·3박 스타카토
    [("F4", 1.0, False, False),
     ("G5", 1.0, True, False),
     ("G5", 1.0, True, False),
     ("C4", 0.5, False, False),
     ("G4", 0.5, False, False)],

    # 12마디 Db7(b5)/F : 2·3박 스타카토. B내추럴은 화음 아래쪽 음임.
    [("G4", 1.0, False, False),
     ("G5", 1.0, True, False),
     ("G5", 1.0, True, False),
     ("F4", 1.0, False, False)],
]

assert all(abs(sum(note[1] for note in bar) - 4.0) < 1e-7 for bar in SCORE_BARS)


def note_events(bpm=BPM):
    """(주파수, 발음 길이 초, 무음 길이 초)를 악보의 박자대로 생성."""
    beat_sec = 60.0 / bpm
    events = []
    for repeat_index in range(REPEAT_COUNT):
      for bar in SCORE_BARS:
        for name, beats, staccato, legato in bar:
            total = beat_sec * beats
            if staccato:
                gap = total * 0.45
            elif legato:
                gap = 0.0
            else:
                gap = min(0.035, total * 0.10)
            events.append((FREQUENCIES[name], total - gap, gap))
    return events


def play_goal_song():
    """GOAL 완료 직후 한 번 연주. 호출 전 반드시 모터를 정지시키세요."""
    from pinkylib import Buzzer
    buzzer = None
    try:
        buzzer = Buzzer()
        buzzer.buzzer_start()
        for hz, on_sec, off_sec in note_events():
            buzzer.set_buzzer_freq(hz)
            buzzer.set_buzzer_duty(DUTY)
            time.sleep(on_sec)
            buzzer.set_buzzer_duty(0)
            if off_sec:
                time.sleep(off_sec)
    except Exception as exc:
        print(f"[GOAL SOUND] {exc}")
    finally:
        if buzzer is not None:
            try:
                buzzer.set_buzzer_duty(0)
            finally:
                buzzer.close()


if __name__ == "__main__":
    play_goal_song()
