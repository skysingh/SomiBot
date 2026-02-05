#!/usr/bin/env python3
"""
Somi the Sommelier - With Emoticons (Static Images)
Face detection + Voice recognition + Emoticon display
Uses udev-assigned audio device names for reliable detection

Version: 14
Platform: Raspberry Pi / Python 3

Based on somi_emoticon_v13
Changes:
- Use udev card names (mic_cmteck, speaker_uac) for reliable audio device detection
- Falls back to name search if udev names not found
"""

__version__ = "14"

import cv2
import os
import subprocess
import time
import json
import array
import signal
import sys
import math
from vosk import Model, KaldiRecognizer

# ═══════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════

# Preferred udev card names (set by /etc/udev/rules.d/99-usb-audio.rules)
SPEAKER_CARD_NAME = "speaker_uac"
MIC_CARD_NAME = "mic_cmteck"

# Fallback: Device names to search for if udev names not found
SPEAKER_NAME_FALLBACK = "UACDemo"
MIC_NAME_FALLBACK = "CMTECK"

# Volume (0-100) - speaker output volume
VOLUME = 70

# Microphone gain - increase if voice is too quiet, decrease if distorted
GAIN = 8.0

# Paths
SOUNDS_DIR = "/home/er/pymycobot/somibot/sounds"
VOSK_MODEL_PATH = os.path.expanduser("~/vosk-model")
EMO_DIR = "/home/er/pymycobot/emo"

# Voice settings
SAMPLE_RATE = 16000
VOICE_TIMEOUT = 10
MAX_RETRIES = 2

WAV_FILES = {
    "ready": "somi_ready.wav",
    "greeting": "somi_greeting.wav",
    "ask_color": "somi_ask_color.wav",
    "repeat": "somi_repeat.wav",
    "red": "somi_red.wav",
    "red_alt": "somi_red_alt.wav",
    "white": "somi_white.wav",
    "white_alt": "somi_white_alt.wav",
    "here_you_go": "somi_here_you_go.wav",
    "goodbye": "somi_goodbye.wav",
    "no_more_wines": "somi_no_more_wines.wav",
}

# Emoticon PNG files (extracted from videos using extract_emoticon_frames.py)
EMOTICONS = {
    "smile": "smile_static.png",
    "happy": "look_happy.png",
    "wink": "wink.png",
    "look_left": "look_left.png",
    "look_right": "look_right.png",
    "sleepy": "sleepy.png",
    "eye_open": "eye_open.png",
    "eye_close": "eye_close.png",
    "fade_in": "fade_in.png",
    "fade_out": "fade_out.png",
    "laugh": "laugh_cry_fadein.png",
}

# These will be set by find_audio_devices()
SPEAKER_DEVICE = None
SPEAKER_CARD = None
MIC_DEVICE = None
MIC_CARD = None

running = True
cap = None

# Emoticon cache and window
emoticon_cache = {}
EMO_WINDOW = "Somi"
emo_window_created = False
quit_check_enabled = False  # Don't check quit until fully started

# ═══════════════════════════════════════════════════════════════
# EMOTICON FUNCTIONS (Static PNG - Fullscreen)
# ═══════════════════════════════════════════════════════════════

def check_quit():
    """Check if q or ESC pressed, return True if should quit"""
    global running
    if not quit_check_enabled:
        cv2.waitKey(1)  # Still process window events
        return False
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:  # q or ESC
        print("\n  Quit key pressed...")
        running = False
        return True
    return False

def create_emoticon_window():
    """Create fullscreen emoticon window"""
    global emo_window_created
    cv2.namedWindow(EMO_WINDOW, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(EMO_WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    emo_window_created = True

def load_emoticons():
    """Pre-load all emoticon images into memory"""
    global emoticon_cache
    loaded = 0
    for name, filename in EMOTICONS.items():
        filepath = os.path.join(EMO_DIR, filename)
        if os.path.exists(filepath):
            img = cv2.imread(filepath)
            if img is not None:
                emoticon_cache[name] = img
                loaded += 1
    return loaded

def show_emoticon(name):
    """Display an emoticon image"""
    global emo_window_created

    if name not in emoticon_cache:
        return False

    if not emo_window_created:
        create_emoticon_window()

    cv2.imshow(EMO_WINDOW, emoticon_cache[name])
    check_quit()  # Check for quit on every emoticon display
    print(f"  😊 {name}")
    return True

# ═══════════════════════════════════════════════════════════════
# AUDIO DEVICE DETECTION
# ═══════════════════════════════════════════════════════════════

def check_card_exists(card_name):
    """Check if a card name exists in /proc/asound/cards"""
    try:
        with open('/proc/asound/cards', 'r') as f:
            content = f.read()
            return card_name in content
    except:
        return False

def find_audio_devices():
    """Find audio devices - prefer udev names, fall back to search"""
    global SPEAKER_DEVICE, SPEAKER_CARD, MIC_DEVICE, MIC_CARD

    # Kill PulseAudio first to free audio devices
    try:
        subprocess.run(['pulseaudio', '-k'],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)  # Give it time to release devices
        print("  ✓ Released audio from PulseAudio")
    except:
        pass

    speaker_found = False
    mic_found = False

    # Try udev card names first (most reliable)
    print("  Checking udev card names...")

    if check_card_exists(SPEAKER_CARD_NAME):
        SPEAKER_DEVICE = f"plughw:{SPEAKER_CARD_NAME},0"
        SPEAKER_CARD = SPEAKER_CARD_NAME
        speaker_found = True
        print(f"    ✓ Speaker: {SPEAKER_CARD_NAME} (udev)")

    if check_card_exists(MIC_CARD_NAME):
        MIC_DEVICE = f"plughw:{MIC_CARD_NAME},0"
        MIC_CARD = MIC_CARD_NAME
        mic_found = True
        print(f"    ✓ Mic: {MIC_CARD_NAME} (udev)")

    # Fall back to name search if udev names not found
    if not speaker_found:
        print(f"  Searching for speaker ({SPEAKER_NAME_FALLBACK})...")
        try:
            result = subprocess.run(['aplay', '-l'], capture_output=True, text=True)
            for line in result.stdout.split('\n'):
                if SPEAKER_NAME_FALLBACK in line and 'card' in line.lower():
                    parts = line.split(':')
                    if parts:
                        card_part = parts[0]
                        card_num = card_part.split()[-1]
                        SPEAKER_DEVICE = f"plughw:{card_num},0"
                        SPEAKER_CARD = card_num
                        speaker_found = True
                        print(f"    ✓ Speaker: card {card_num} (fallback)")
                        break
        except Exception as e:
            print(f"    Error: {e}")

    if not mic_found:
        print(f"  Searching for mic ({MIC_NAME_FALLBACK})...")
        try:
            result = subprocess.run(['arecord', '-l'], capture_output=True, text=True)
            for line in result.stdout.split('\n'):
                if MIC_NAME_FALLBACK in line and 'card' in line.lower():
                    parts = line.split(':')
                    if parts:
                        card_part = parts[0]
                        card_num = card_part.split()[-1]
                        MIC_DEVICE = f"plughw:{card_num},0"
                        MIC_CARD = card_num
                        mic_found = True
                        print(f"    ✓ Mic: card {card_num} (fallback)")
                        break
        except Exception as e:
            print(f"    Error: {e}")

    if not speaker_found:
        print(f"    ✗ Speaker not found")
    if not mic_found:
        print(f"    ✗ Mic not found")

    return speaker_found and mic_found

# ═══════════════════════════════════════════════════════════════
# VOLUME CONTROL
# ═══════════════════════════════════════════════════════════════

def set_volume(level):
    """Set speaker volume (0-100)"""
    if not SPEAKER_CARD:
        return False
    level = max(0, min(100, level))
    try:
        subprocess.run(
            ['amixer', '-c', SPEAKER_CARD, 'set', 'PCM', f'{level}%'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        return True
    except:
        return False

def get_volume():
    """Get current volume level"""
    if not SPEAKER_CARD:
        return -1
    try:
        result = subprocess.run(
            ['amixer', '-c', SPEAKER_CARD, 'get', 'PCM'],
            capture_output=True, text=True
        )
        for line in result.stdout.split('\n'):
            if '%' in line:
                start = line.find('[') + 1
                end = line.find('%')
                if start > 0 and end > start:
                    return int(line[start:end])
    except:
        pass
    return -1

def set_mic_volume(level=100):
    """Set microphone capture volume (0-100)"""
    if not MIC_CARD:
        return False
    level = max(0, min(100, level))
    try:
        subprocess.run(
            ['amixer', '-c', MIC_CARD, 'set', 'Mic', f'{level}%'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        return True
    except:
        return False

def get_mic_volume():
    """Get current mic capture level"""
    if not MIC_CARD:
        return -1
    try:
        result = subprocess.run(
            ['amixer', '-c', MIC_CARD, 'get', 'Mic'],
            capture_output=True, text=True
        )
        for line in result.stdout.split('\n'):
            if '%' in line:
                start = line.find('[') + 1
                end = line.find('%')
                if start > 0 and end > start:
                    return int(line[start:end])
    except:
        pass
    return -1

# ═══════════════════════════════════════════════════════════════
# SIGNAL HANDLER
# ═══════════════════════════════════════════════════════════════

def cleanup_and_exit(sig=None, frame=None):
    global running, cap
    print("\n\n✓ Stopping...")
    running = False
    if cap:
        cap.release()
    cv2.destroyAllWindows()
    print("✓ Goodbye!")
    os._exit(0)

signal.signal(signal.SIGINT, cleanup_and_exit)

# ═══════════════════════════════════════════════════════════════
# AUDIO
# ═══════════════════════════════════════════════════════════════

def apply_gain(data, gain):
    """Apply gain to audio data and return (boosted_data, rms_level)"""
    samples = array.array('h', data)
    boosted = []
    sum_sq = 0
    for s in samples:
        b = int(s * gain)
        b = max(-32768, min(32767, b))
        boosted.append(b)
        sum_sq += b * b

    # Calculate RMS level (0-100 scale)
    if len(samples) > 0:
        rms = math.sqrt(sum_sq / len(samples))
        level = min(100, int(rms / 327.67))  # Scale to 0-100
    else:
        level = 0

    return array.array('h', boosted).tobytes(), level

def level_bar(level, width=20):
    """Create a visual level bar"""
    filled = int(level * width / 100)
    bar = '█' * filled + '░' * (width - filled)
    return f"[{bar}]"

def play_wav(wav_key):
    if not running or not SPEAKER_DEVICE:
        return False
    filename = WAV_FILES.get(wav_key)
    if not filename:
        return False
    filepath = os.path.join(SOUNDS_DIR, filename)
    if not os.path.exists(filepath):
        return False
    try:
        subprocess.run(['aplay', '-D', SPEAKER_DEVICE, filepath],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
        print(f"  🔊 {wav_key}")
        return True
    except:
        return False

def listen(vosk_model, timeout=VOICE_TIMEOUT):
    if not running or not MIC_DEVICE:
        return None
    print(f"  🎤 Listening ({timeout}s)...")

    rec = KaldiRecognizer(vosk_model, SAMPLE_RATE)
    rec.SetWords(True)

    process = subprocess.Popen(
        ['arecord', '-D', MIC_DEVICE, '-f', 'S16_LE', '-r', str(SAMPLE_RATE), '-c', '1', '-t', 'raw', '-q'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
    )

    text = None
    start_time = time.time()
    silence_start = None
    last_partial = ""

    try:
        while running and (time.time() - start_time < timeout):
            # Check for quit key during listening
            if check_quit():
                break

            data = process.stdout.read(2000)
            if len(data) == 0:
                break

            boosted, level = apply_gain(data, GAIN)
            remaining = max(0, timeout - (time.time() - start_time))

            if rec.AcceptWaveform(boosted):
                result = json.loads(rec.Result())
                if result.get('text'):
                    text = result['text']
                    print(f"\r  💬 \"{text}\"" + " "*30)
                    break
            else:
                partial = json.loads(rec.PartialResult())
                partial_text = partial.get('partial', '')

                if partial_text:
                    last_partial = partial_text
                    silence_start = None
                    # Show level + partial text
                    print(f"\r  {level_bar(level)} 💭 {partial_text:<25} [{remaining:.0f}s]", end='', flush=True)
                elif last_partial:
                    if silence_start is None:
                        silence_start = time.time()
                    elif time.time() - silence_start > 1.2:  # Increased from 0.8 to 1.2
                        text = last_partial
                        print(f"\r  💬 \"{text}\"" + " "*30)
                        break
                    else:
                        # Show level while waiting for silence timeout
                        print(f"\r  {level_bar(level)} 💭 {last_partial:<25} [{remaining:.0f}s]", end='', flush=True)
                else:
                    # Show just level when no speech detected
                    print(f"\r  {level_bar(level)} (waiting...){' '*20} [{remaining:.0f}s]", end='', flush=True)

        if not text and running:
            final = json.loads(rec.FinalResult())
            if final.get('text'):
                text = final['text']
                print(f"\r  💬 \"{text}\"" + " "*30)
            elif last_partial:
                text = last_partial
                print(f"\r  💬 \"{text}\"" + " "*30)
            else:
                print(f"\r  ⏱ No response" + " "*30)
    except:
        pass
    finally:
        process.terminate()
        process.wait()

    return text

# ═══════════════════════════════════════════════════════════════
# HELPERS - Word matching with common misrecognitions
# ═══════════════════════════════════════════════════════════════

def contains_yes(text):
    if not text: return False
    words = ["yes", "yeah", "sure", "okay", "ok", "please", "yep", "yup",
             "guess", "yea", "ya", "uh huh", "huh", "uh", "mhm", "mm hmm"]
    return any(w in text.lower() for w in words)

def contains_no(text):
    if not text: return False
    words = ["no", "nope", "nah", "no thanks", "not"]
    text_lower = text.lower()
    if text_lower.strip() in ["no", "nope", "nah"]:
        return True
    return any(w in text_lower for w in ["no thank", "nope", "nah", "not really"])

def contains_red(text):
    if not text: return False
    words = ["red", "read", "bread", "fred", "said red", "the red", "rent"]
    return any(w in text.lower() for w in words)

def contains_white(text):
    if not text: return False
    words = ["white", "wide", "light", "quite", "wight"]
    text_lower = text.lower()
    if "right" in text_lower and "white" not in text_lower:
        if text_lower.strip() in ["right", "the right"]:
            return True
    return any(w in text_lower for w in words)

# ═══════════════════════════════════════════════════════════════
# DIALOG (with emoticons)
# ═══════════════════════════════════════════════════════════════

def offer_wine(vosk_model, primary_wav, alt_wav):
    if not running: return False

    print("\n  [Offering wine]")
    show_emoticon("happy")
    play_wav(primary_wav)

    show_emoticon("look_left")
    response = listen(vosk_model)

    if contains_yes(response):
        print("\n  [Accepted!]")
        show_emoticon("wink")
        play_wav("here_you_go")
        return True
    elif contains_no(response):
        print("\n  [Trying alternative]")
        show_emoticon("smile")
        play_wav(alt_wav)

        show_emoticon("look_right")
        response = listen(vosk_model)
        if contains_yes(response):
            print("\n  [Accepted alternative!]")
            show_emoticon("wink")
            play_wav("here_you_go")
            return True
        else:
            print("\n  [Declined]")
            show_emoticon("sleepy")
            play_wav("no_more_wines")
            return False
    else:
        print("\n  [Serving anyway]")
        show_emoticon("wink")
        play_wav("here_you_go")
        return True

def run_dialog(vosk_model):
    if not running: return

    # Step 1: Greeting
    print("\n  [Greeting]")
    show_emoticon("smile")
    play_wav("greeting")

    show_emoticon("look_left")
    response = listen(vosk_model)

    if not contains_yes(response):
        print("\n  [Goodbye]")
        show_emoticon("eye_close")
        play_wav("goodbye")
        return

    # Step 2: Ask color
    for attempt in range(MAX_RETRIES + 1):
        if not running: return
        print(f"\n  [Ask color - attempt {attempt+1}]")
        show_emoticon("happy")
        play_wav("ask_color")

        show_emoticon("look_right")
        response = listen(vosk_model)

        if contains_red(response):
            print("\n  [Red branch]")
            offer_wine(vosk_model, "red", "red_alt")
            return
        elif contains_white(response):
            print("\n  [White branch]")
            offer_wine(vosk_model, "white", "white_alt")
            return
        else:
            if attempt < MAX_RETRIES:
                print("\n  [Unclear - retry]")
                show_emoticon("sleepy")
                play_wav("repeat")
            else:
                print("\n  [Default to red]")
                offer_wine(vosk_model, "red", "red_alt")
                return

# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main():
    global cap, running, quit_check_enabled

    print()
    print("="*50)
    print(f"  🍷 SOMI THE SOMMELIER v{__version__} 🍷")
    print("      (with emoticons)")
    print("="*50)
    print("  Ctrl+C to quit")
    print()

    # Find audio devices
    print("Finding audio devices...")
    if find_audio_devices():
        print(f"  ✓ Speaker: {SPEAKER_DEVICE}")
        print(f"  ✓ Microphone: {MIC_DEVICE}")
    else:
        print("  ✗ Audio devices not found!")
        print(f"    Looking for speaker: {SPEAKER_CARD_NAME} or {SPEAKER_NAME_FALLBACK}")
        print(f"    Looking for mic: {MIC_CARD_NAME} or {MIC_NAME_FALLBACK}")
        print("    Check USB connections and run: aplay -l && arecord -l")
        print("    Or check udev rules: cat /etc/udev/rules.d/99-usb-audio.rules")
        return

    # Set speaker volume
    print(f"Setting speaker volume to {VOLUME}%...")
    set_volume(VOLUME)
    current_vol = get_volume()
    if current_vol >= 0:
        print(f"  ✓ Speaker: {current_vol}%")
    else:
        print(f"  ⚠ Could not verify speaker volume")

    # Set mic volume to max
    print("Setting mic capture to 100%...")
    set_mic_volume(100)
    mic_vol = get_mic_volume()
    if mic_vol >= 0:
        print(f"  ✓ Mic: {mic_vol}%")
    else:
        print(f"  ⚠ Could not verify mic volume")

    # Check sound files
    print("Checking sound files...")
    for f in WAV_FILES.values():
        if not os.path.exists(os.path.join(SOUNDS_DIR, f)):
            print(f"  ✗ Missing: {f}")
            return
    print("  ✓ Sounds OK")

    # Load emoticon images
    print("Loading emoticon images...")
    loaded = load_emoticons()
    if loaded == 0:
        print("  ✗ No emoticon images found!")
        print(f"    Expected PNG files in: {EMO_DIR}")
        print("    Run: python3 extract_emoticon_frames.py")
        return
    elif loaded < len(EMOTICONS):
        print(f"  ⚠ Loaded {loaded}/{len(EMOTICONS)} emoticons")
    else:
        print(f"  ✓ Loaded {loaded} emoticons")

    # Load Vosk
    print("Loading Vosk...")
    vosk_model = Model(VOSK_MODEL_PATH)
    print("  ✓ Vosk OK")

    # Face detector
    cascade_paths = [
        '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
        '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml',
        '/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
        '/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml',
    ]
    try:
        cascade_paths.insert(0, cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    except AttributeError:
        pass

    cascade_file = None
    for path in cascade_paths:
        if os.path.exists(path):
            cascade_file = path
            break

    if not cascade_file:
        print("  ✗ Could not find haarcascade_frontalface_default.xml")
        print("    Try: sudo apt install opencv-data")
        return

    face_cascade = cv2.CascadeClassifier(cascade_file)
    if face_cascade.empty():
        print(f"  ✗ Failed to load cascade from {cascade_file}")
        return
    print(f"  ✓ Face detector OK")

    # Camera
    print("Opening camera...")
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("  Camera 2 failed, trying 0...")
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("  ✗ No camera!")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("  ✓ Camera OK")

    print()
    print("="*50)
    print("READY - Waiting for face")
    print(f"Speaker: {VOLUME}% | Mic: 100% | Gain: {GAIN}x")
    print("Press 'q' or ESC in emoticon window to quit")
    print("Or Ctrl+C in terminal")
    print("="*50)
    print()

    # NOW create fullscreen emoticon window and show smile
    create_emoticon_window()
    show_emoticon("smile")

    # Say "Somi is ready"
    play_wav("ready")

    detection_count = 0
    cooldown_until = 0

    # Give camera time to stabilize
    print("  Warming up camera...")
    for _ in range(10):
        cap.read()
        time.sleep(0.1)
    print("  ✓ Camera ready")

    # NOW enable quit key detection
    quit_check_enabled = True
    print("  ✓ Press 'q' or ESC to quit")

    while running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        frame = cv2.flip(frame, 0)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5, minSize=(30, 30))

        now = time.time()

        if len(faces) > 0 and now >= cooldown_until:
            detection_count += 1
            print("="*50)
            print(f"🍷 CUSTOMER #{detection_count}")
            print("="*50)

            # Release camera during dialog to free CPU
            cap.release()

            run_dialog(vosk_model)

            # Return to idle smile
            show_emoticon("smile")

            print("\n  ✓ Complete")
            print("="*50)
            print()

            # Re-open camera for next customer
            cap = cv2.VideoCapture(2)
            if not cap.isOpened():
                cap = cv2.VideoCapture(0)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            cooldown_until = time.time() + 5.0

        # Check for quit key (q or ESC)
        if check_quit():
            break

        time.sleep(0.05)

    cleanup_and_exit()

if __name__ == "__main__":
    main()
