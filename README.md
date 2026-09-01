# A Real-Time Human-Computer Interaction Using Hand Gestures in OpenCV

A virtual mouse and virtual keyboard driven entirely by hand gestures in front of a webcam. No
wearables, no depth camera — **MediaPipe** tracks 21 hand landmarks, the finger-state pattern is
read as a gesture, and the gesture is dispatched to the OS as a real mouse, keyboard or
application action.

![Python](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?logo=opencv&logoColor=white)
![MediaPipe](https://img.shields.io/badge/MediaPipe-0097A7?logo=google&logoColor=white)

## Why

Conventional interaction assumes a mouse and a keyboard, which assumes a user who can operate
one. This project replaces both with gestures, aimed at making a desktop usable for people who
cannot comfortably use the standard hardware — and it works with an ordinary webcam.

## How it works

`HandTrackingModule2.py` wraps MediaPipe Hands into a `HandDetector` that returns, per hand: the
21 landmark points, a bounding box, a centre point, handedness, and — through `fingersUp()` — a
five-element binary vector of which fingers are extended.

`Project_Phase_1_Final.py` is the controller. It reads that vector every frame and runs a small
state machine over it. The important design decision is that **gestures are transitions, not
poses**: a click is not "index finger up", it is "index finger up *arriving from* the V pose".
Requiring a previous state means an incidental hand shape passing through the frame does not fire
an action, and a `buttonDelay` counter debounces the ones that do.

**Neutral** — all five fingers open — is the home position that most shortcuts must be entered
from.

### Mouse mode

| Gesture | Fingers | Action |
| --- | --- | --- |
| Open palm | all 5 | Neutral (home state) |
| Index only | `[0,1,0,0,0]` | Move the pointer |
| V sign | index + middle | Arm the click state |
| V → index only | | Left click (right hand) / right click (left hand) |
| V → middle only | | Right click (right hand) / left click (left hand) |
| V, fingers pinched together | ratio < 1.1 | Double click |
| V → fist | `[0,0,0,0,0]` | Press and hold — drag; reopening to V releases |
| Neutral → fist | | Screenshot, saved to `Pictures/Screenshots/` |

Pointer motion is mapped from a reduced interaction rectangle inside the frame, so the user covers
the whole screen with a small hand movement instead of reaching across the camera's field of view.
Positions are exponentially smoothed (`smoothening = 7`) to take the jitter out of the cursor.

### Three fingers — continuous controls

With `[0,1,1,1,0]` the hand becomes an analogue control, and the direction of motion selects what
it controls:

| Hand | Motion | Controls |
| --- | --- | --- |
| Right | horizontal | Scroll left / right |
| Right | vertical | Scroll up / down |
| Left | horizontal | Volume up / down |
| Left | vertical | Brightness up / down |

Holding the gesture accelerates it — the step size doubles the longer you stay in the same
direction (capped at 4x for volume), so both a nudge and a long sweep are comfortable.

### Application shortcuts

| Gesture | Right hand | Left hand |
| --- | --- | --- |
| Index + pinky | Open Chrome | — |
| Thumb only | New tab (Ctrl+T) | Close tab (Ctrl+W) |
| Thumb + pinky | Open WhatsApp | Open Calculator |

### Keyboard mode

A toggle gesture switches the app into a virtual QWERTY keyboard drawn over the camera feed
(digits, letters, plus `<--`, `CAPS`, `SPACE` and `ENTER`). Pinching over a key presses it through
`pynput`, and the capture resolution is raised to 1280x720 in this mode so the key targets are big
enough to hit reliably.

## Running it

```bash
pip install opencv-python mediapipe numpy autopy pyautogui pynput cvzone \
            screen-brightness-control AppOpener
python Project_Phase_1_Final.py
```

Both files must sit in the same directory. Press `q` to quit.

> **Platform note** — this targets **Windows**. It uses `cv2.CAP_DSHOW`, the `USERPROFILE`
> environment variable for the screenshot path, and `AppOpener` for launching applications. The
> tracking and gesture logic are portable; the OS-action layer would need swapping out for
> macOS or Linux.

## Repository layout

```
HandTrackingModule2.py    # MediaPipe wrapper: landmarks, handedness, fingersUp, distances
Project_Phase_1_Final.py  # gesture state machine, mouse/keyboard/app actions, virtual keyboard
Project Report.pdf        # full 63-page write-up
Project PPT.pptx
```

## Known limitations

Documented in the report and worth repeating: frame rate is the binding constraint. On an
underpowered machine, in poor lighting, or with a cluttered background, MediaPipe's per-frame
inference slows down, and a low FPS shows up directly as lag between the gesture and the action.
Good lighting and a plain background make a large difference.

## Team

B.Tech project in Computer Science and Engineering at **Amrita School of Computing, Bangalore**
(Amrita Vishwa Vidyapeetham), December 2022, guided by Mr. Niranjan D K.

- K. Vishnu Sainadh
- K. Satwik
- V. Ashrith

Full design, testing and results are in [`Project Report.pdf`](Project%20Report.pdf).
