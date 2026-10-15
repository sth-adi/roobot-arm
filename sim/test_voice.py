import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dashboard"))
import voice  # noqa: E402

LOUD = (b"\x00\x20" * 16000)                   # one second of a steady tone, well above the silence threshold


class FakeModel:
    """Stands in for Whisper: returns the given segments and remembers how it was called."""
    def __init__(self, *segs):
        self.segs, self.calls = segs, []

    def transcribe(self, samples, **kw):
        self.calls.append((len(samples), kw))
        return iter([SimpleNamespace(text=t, no_speech_prob=p) for t, p in self.segs]), None


class AudioChecksTest(unittest.TestCase):
    def test_refuses_unusable_audio(self):
        for raw in (b"", b"\x00", LOUD[:-1], b"\x00\x20" * 100, b"\x00\x20" * (voice.MAX_BYTES // 2 + 1)):
            with self.assertRaises(voice.VoiceError):
                voice.pcm_to_float(raw)

    def test_samples_are_scaled_to_one(self):
        s = voice.pcm_to_float((32767).to_bytes(2, "little", signed=True) * 8000 + (-32768).to_bytes(2, "little", signed=True) * 8000)
        self.assertAlmostEqual(float(s.max()), 32767 / 32768, places=4)
        self.assertEqual(float(s.min()), -1.0)


class TranscribeTest(unittest.TestCase):
    def test_text_comes_back_cleaned(self):
        m = FakeModel((" Move up", 0.0), (" 2 cm. ", 0.0))
        r = voice.transcribe(LOUD, m)
        self.assertEqual((r["text"], r["stop"]), ("Move up 2 cm.", False))
        self.assertEqual(m.calls[0][0], 16000)
        self.assertEqual(m.calls[0][1]["language"], "en")

    def test_silence_never_reaches_the_model(self):
        m = FakeModel(("Thank you.", 0.0))
        self.assertEqual(voice.transcribe(b"\x00\x00" * 16000, m)["text"], "")
        self.assertEqual(m.calls, [])

    def test_segments_the_model_thinks_are_not_speech_are_dropped(self):
        self.assertEqual(voice.transcribe(LOUD, FakeModel(("Thanks for watching!", 0.9)))["text"], "")

    def test_the_hint_repeated_back_is_not_speech(self):
        self.assertEqual(voice.transcribe(LOUD, FakeModel((voice.HINT, 0.0)))["text"], "")

    def test_only_a_short_plain_stop_is_sent_without_confirming(self):
        self.assertTrue(voice.transcribe(LOUD, FakeModel(("Stop!", 0.0)))["stop"])
        self.assertTrue(voice.transcribe(LOUD, FakeModel(("stop the arm", 0.0)))["stop"])
        self.assertFalse(voice.transcribe(LOUD, FakeModel(("please do not stop until the key is on the tray", 0.0)))["stop"])
        self.assertFalse(voice.transcribe(LOUD, FakeModel(("go home", 0.0)))["stop"])
        self.assertFalse(voice.transcribe(LOUD, FakeModel(("", 0.0)))["stop"])

    def test_text_is_capped_like_a_typed_request(self):
        self.assertEqual(len(voice.transcribe(LOUD, FakeModel(("a " * 600, 0.0)))["text"]), 500)


if __name__ == "__main__":
    unittest.main()
