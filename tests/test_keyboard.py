import unittest
from greenhouse.keyboard import KeyDecoder, velocities


class KeyboardTests(unittest.TestCase):
    def test_arrows_fragmented_sequences_and_standalone_escape(self):
        decoder = KeyDecoder()
        self.assertEqual(decoder.feed("\x1b", 1.0), [])
        self.assertEqual(decoder.feed("[A", 1.02), ["up"])
        self.assertEqual(decoder.feed("\x1b[B\x1b[C\x1b[D", 2.0), ["down", "right", "left"])
        self.assertEqual(decoder.feed("\x1bOA", 3), ["up"])
        self.assertEqual(decoder.feed("\x1b", 4), [])
        self.assertEqual(decoder.feed("", 4.07), ["quit"])
        self.assertEqual(decoder.feed("\x1b[1;5A", 5), [])
        self.assertEqual(decoder.feed("\x1b[15~", 6), [])
        self.assertEqual(decoder.feed("\x1bq", 7), ["quit"])

    def test_lift_camera_and_q_is_not_quit(self):
        self.assertEqual(KeyDecoder().feed("aQc ", 0), ["a", "q", "c", "stop"])
        self.assertEqual(velocities({"up", "a"}, 0.2, 0.5), (0.2, 0.0, 1.0))
        self.assertEqual(velocities({"up", "down", "left", "right", "a", "q"}, 0.2, 0.5), (0.0, 0.0, 0.0))
