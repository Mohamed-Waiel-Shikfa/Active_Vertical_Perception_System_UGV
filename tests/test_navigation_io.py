import json
import tempfile
import unittest
import threading
import time
from unittest.mock import patch
from pathlib import Path

from greenhouse.navigation_io import NavigationJournal


class NavigationIoTests(unittest.TestCase):
    def test_ordered_trace_and_complete_status_flush(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            writer=NavigationJournal(root,root/"live/status.json")
            writer.write('{"step":1}\n')
            writer.status({"ready":False,"phase":"Starting"})
            writer.write('{"step":2}\n')
            writer.status({"ready":True,"phase":"Ready"})
            writer.close()
            self.assertEqual((root/"navigation-trace.jsonl").read_text(),'{"step":1}\n{"step":2}\n')
            self.assertEqual(json.loads((root/"navigation-status.json").read_text()),{"ready":True,"phase":"Ready"})
            self.assertEqual(json.loads((root/"live/status.json").read_text()),{"ready":True,"phase":"Ready"})

    def test_slow_durable_mirror_does_not_block_local_status(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            blocked,released=threading.Event(),threading.Event()
            original=Path.write_text
            def slow(path,text,*args,**kwargs):
                if path==root/"navigation-status.tmp":
                    blocked.set()
                    if not released.wait(2):
                        raise OSError("test mirror deadline")
                return original(path,text,*args,**kwargs)
            with patch.object(Path,"write_text",slow):
                writer=NavigationJournal(root,root/"live/status.json")
                try:
                    writer.status({"sequence":1})
                    self.assertTrue(blocked.wait(1))
                    writer.status({"sequence":2})
                    deadline=time.monotonic()+1
                    latest={}
                    while time.monotonic()<deadline:
                        if (root/"live/status.json").exists():
                            latest=json.loads((root/"live/status.json").read_text())
                            if latest=={"sequence":2}:break
                        time.sleep(.01)
                    self.assertEqual(latest,{"sequence":2})
                finally:
                    released.set()
                    writer.close()
