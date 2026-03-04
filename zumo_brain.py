import cv2
import serial
import time
import mediapipe as mp
import pygame
import os

# --- CONFIGURATION ---
ARDUINO_PORT = 'COM3'  # Ensure this matches your port
BAUD_RATE = 9600
BUFFER_SECONDS = 1.0
AUDIO_FILE = 'success.mp3'
AUDIO_COOLDOWN = 12.0

print(f"Attempting to connect to {ARDUINO_PORT}...")


# --- SETUP SERIAL FUNCTION ---
def connect_arduino():
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Allow Arduino to reset
        print(f"SUCCESS: Connected to Arduino on {ARDUINO_PORT}")
        return ser
    except Exception as e:
        print(f"ERROR: Could not connect to Arduino. {e}")
        return None


arduino = connect_arduino()


# --- HELPER: ROBUST SERIAL WRITE ---
def send_command(command_bytes):
    global arduino
    if arduino is None:
        # Try to reconnect occasionally if lost
        arduino = connect_arduino()
        if arduino is None: return

    try:
        arduino.write(command_bytes)
    except (serial.SerialException, OSError, PermissionError) as e:
        print(f"CONNECTION LOST: {e}")
        print("Attempting to reconnect...")
        try:
            arduino.close()
        except:
            pass
        arduino = None
        # Immediate retry
        arduino = connect_arduino()


# --- SETUP AUDIO ---
try:
    pygame.mixer.init()
    if os.path.exists(AUDIO_FILE):
        sound_effect = pygame.mixer.Sound(AUDIO_FILE)
        print("Audio system initialized.")
    else:
        print(f"WARNING: Audio file '{AUDIO_FILE}' not found.")
        sound_effect = None
except Exception as e:
    print(f"Audio Error: {e}")
    sound_effect = None

# --- SETUP COMPUTER VISION ---
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# --- LOGIC VARIABLES ---
last_seen_time = time.time()
last_audio_time = 0


def is_thumbs_up(hand_landmarks):
    thumb_tip = hand_landmarks.landmark[4]
    thumb_ip = hand_landmarks.landmark[3]
    index_tip = hand_landmarks.landmark[8]
    index_pip = hand_landmarks.landmark[6]
    middle_tip = hand_landmarks.landmark[12]
    middle_pip = hand_landmarks.landmark[10]
    ring_tip = hand_landmarks.landmark[16]
    ring_pip = hand_landmarks.landmark[14]
    pinky_tip = hand_landmarks.landmark[20]
    pinky_pip = hand_landmarks.landmark[18]

    thumb_is_up = thumb_tip.y < thumb_ip.y
    index_folded = index_tip.y > index_pip.y
    middle_folded = middle_tip.y > middle_pip.y
    ring_folded = ring_tip.y > ring_pip.y
    pinky_folded = pinky_tip.y > pinky_pip.y

    return thumb_is_up and index_folded and middle_folded and ring_folded and pinky_folded


print("Tracking started. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret: break

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # --- 1. FACE TRACKING ---
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=8, minSize=(60, 60))

    if len(faces) > 0:
        last_seen_time = time.time()
        target_face = sorted(faces, key=lambda f: f[2] * f[3], reverse=True)[0]
        (x, y, w, h) = target_face
        center_x = x + (w // 2)
        center_y = y + (h // 2)

        data = f"{center_x},{center_y}\n"
        send_command(data.encode('utf-8'))

        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, "LOCKED", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        time_since_loss = time.time() - last_seen_time
        if time_since_loss > BUFFER_SECONDS:
            send_command(b"CLOSE\n")
            cv2.putText(frame, "SLEEPING", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            remaining = round(BUFFER_SECONDS - time_since_loss, 1)
            cv2.putText(frame, f"SEARCHING... ({remaining}s)", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

    # --- 2. HAND TRACKING ---
    result = hands.process(rgb_frame)
    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            if is_thumbs_up(hand_landmarks):
                cv2.putText(frame, "THUMBS UP!", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                current_time = time.time()
                if sound_effect and (current_time - last_audio_time > AUDIO_COOLDOWN):
                    try:
                        sound_effect.play()
                        print("Audio Playing...")
                        # Send SPEAK command safely
                        send_command(b"SPEAK\n")
                        print("Sent SPEAK signal")
                        last_audio_time = current_time
                    except Exception as e:
                        print(f"Play Error: {e}")

    cv2.imshow('Animatronic Eye Tracking', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
if arduino:
    arduino.close()
cv2.destroyAllWindows()
