"""Speech to text for the Chat page: a short recording from the browser becomes the text that is then asked like a typed request.

Runs a small Whisper model on this PC (faster-whisper, CPU). Nothing is sent to any service after the one-time model download, and
no API key or cost is involved. The audio is 16 kHz, mono, 16-bit little-endian PCM, which the page makes in the browser, so no
audio decoder or ffmpeg is needed here. The text is only a request: it goes through the same choosing and checks as typed text.
"""
import importlib.util
import os
import re
import threading
import time

try:
    from preflight import is_stop
except ImportError:                                    # imported as dashboard.voice
    from dashboard.preflight import is_stop

SAMPLE_RATE = 16000
MIN_SECONDS = 0.4
MAX_SECONDS = 15
MAX_BYTES = SAMPLE_RATE * 2 * MAX_SECONDS
MODEL_NAME = os.environ.get("WHISPER_MODEL", "base.en")          # tiny.en is faster and a little worse; small.en is slower and better
QUIET = 0.01                                                      # a recording whose loudest sample is below this (of 1.0) is silence, not speech
NO_SPEECH_P = 0.6
HINT = ("Robot arm commands: stop, go home, magnet on, magnet off, move up 2 cm, move left 5 mm, "
        "put the key from box 2 on the tray, bring key 3 back, where is key 1?")   # nudges the model towards the words this desk uses
_model = [None]
_lock = threading.Lock()


class VoiceError(ValueError):
    pass


def available():
    """True when faster-whisper is installed (the page hides the microphone button otherwise)."""
    return importlib.util.find_spec("faster_whisper") is not None


def pcm_to_float(raw):
    """Little-endian 16-bit mono PCM bytes -> float32 samples in [-1, 1]; refuses empty, odd-sized, too short or too long audio."""
    import numpy as np
    if not raw or len(raw) % 2:
        raise VoiceError("that recording is empty or damaged")
    if len(raw) > MAX_BYTES:
        raise VoiceError("that recording is longer than %d seconds" % MAX_SECONDS)
    if len(raw) < SAMPLE_RATE * 2 * MIN_SECONDS:
        raise VoiceError("that recording is too short")
    return np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0


def _load():
    if _model[0] is None:
        from faster_whisper import WhisperModel      # downloads the model once (about 150 MB for base.en), then loads it from the cache
        _model[0] = WhisperModel(MODEL_NAME, device="cpu", compute_type="int8")
    return _model[0]


def clean(text):
    return re.sub(r"\s+", " ", text).strip()[:500]


def is_stop_phrase(text):
    """A short, plain stop command may be sent straight away; anything longer waits for the person to confirm."""
    return len(re.findall(r"[a-z]+", text.lower())) <= 4 and is_stop(text)


def transcribe(raw, model=None):
    """{"text", "stop", "ms"} for a recording. Empty text means nothing was heard. Raises VoiceError for unusable audio."""
    samples = pcm_to_float(raw)
    t0 = time.time()
    if float(abs(samples).max()) < QUIET:
        return {"text": "", "stop": False, "ms": 0}
    with _lock:                                       # one recording at a time; the model is not shared between threads
        segments, _ = (model or _load()).transcribe(samples, language="en", beam_size=3, vad_filter=True, initial_prompt=HINT,
                                                    condition_on_previous_text=False)
        text = clean(" ".join(s.text for s in segments if getattr(s, "no_speech_prob", 0.0) < NO_SPEECH_P))
    if len(text) > 40 and text.lower() in HINT.lower():       # Whisper sometimes repeats its hint when nothing was said: that is not speech
        text = ""
    return {"text": text, "stop": bool(text) and is_stop_phrase(text), "ms": int((time.time() - t0) * 1000)}
