import tempfile
import unittest
from pathlib import Path

from greenhouse.atomic import replace


class AtomicTests(unittest.TestCase):
    def test_replace_swaps_the_whole_file(self):
        with tempfile.TemporaryDirectory() as folder:
            source, destination = Path(folder) / "new.tmp", Path(folder) / "state.json"
            destination.write_text("old")
            source.write_text("new")
            replace(source, destination)
            self.assertEqual(destination.read_text(), "new")
            self.assertFalse(source.exists())

    def test_errors_surface(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(FileNotFoundError):
                replace(Path(folder) / "missing", Path(folder) / "state.json")


if __name__ == "__main__":
    unittest.main()
