"""Publish live status locally without waiting for the durable OneDrive mirror."""
import json
import os
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any

from greenhouse.atomic import replace


class NavigationJournal:
    def __init__(self,directory,cache_path=None):
        self.directory=directory
        self.cache_path=Path(cache_path or os.environ["GREENHOUSE_NAV_CACHE"])
        self.cache_path.parent.mkdir(parents=True,exist_ok=True)
        self.queue=queue.Queue(maxsize=512)
        self.condition=threading.Condition()
        self.local_latest=None
        self.durable_latest=None
        self.closing=False
        self.local_error=None
        self.durable_error=None
        self.local_thread=threading.Thread(target=self.run_local,daemon=True)
        self.durable_thread=threading.Thread(target=self.run_durable,daemon=True)
        self.local_thread.start()
        self.durable_thread.start()

    @property
    def persistence_error(self) -> str | None:
        """Text of the durable-mirror failure, or None while the mirror is healthy."""
        return str(self.durable_error) if self.durable_error else None

    def fail_durable(self,error: Exception) -> None:
        """Stop durable evidence (local status keeps working) and report `error` on stderr."""
        self.durable_error=error
        print(f"Navigation durable evidence unavailable: {error}. Live feedback and local status remain independent.",
              file=sys.stderr,flush=True)

    def write(self,text: str) -> None:
        """Queue one newline-terminated line for navigation-trace.jsonl without blocking.

        Ignored once durable evidence has failed; a full 512-entry queue marks it failed.
        """
        if self.durable_error is not None:
            return
        try:
            self.queue.put_nowait(("trace",text))
        except queue.Full:
            self.fail_durable(RuntimeError("Durable trace queue is full; subsequent trace is not persisted"))

    def status(self,value: Any) -> None:
        """Publish JSON-serialisable `value` as the newest status for the local cache and the durable mirror.

        Raises RuntimeError if the local writer has already failed.
        """
        if self.local_error is not None:
            raise RuntimeError(f"Local navigation status failed: {self.local_error}") from self.local_error
        text=json.dumps(value)+"\n"
        with self.condition:
            self.local_latest=text
            self.durable_latest=text
            self.condition.notify_all()

    def run_local(self) -> None:
        """Writer thread: atomically replace the local cache file with each newest status until close."""
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda:self.local_latest is not None or self.closing)
                    if self.local_latest is None and self.closing:
                        return
                    value,self.local_latest=self.local_latest,None
                temporary=self.cache_path.with_suffix(".tmp")
                temporary.write_text(value)
                replace(temporary,self.cache_path)
        except OSError as error:
            self.local_error=error
            print(f"Local navigation status failed: {error}",file=sys.stderr,flush=True)

    def mirror_status(self) -> None:
        """Atomically write the newest unmirrored status, if any, to `directory`/navigation-status.json."""
        with self.condition:
            value,self.durable_latest=self.durable_latest,None
        if value is not None:
            temporary=self.directory/"navigation-status.tmp"
            temporary.write_text(value)
            replace(temporary,self.directory/"navigation-status.json")

    def run_durable(self) -> None:
        """Writer thread: append queued trace lines and mirror status at most once per wall-clock second."""
        try:
            last_mirror=0.0
            with (self.directory/"navigation-trace.jsonl").open("a",buffering=1) as trace:
                while True:
                    try:
                        kind,value=self.queue.get(timeout=.2)
                    except queue.Empty:
                        kind,value=None,None
                    if kind=="close":
                        self.mirror_status()
                        return
                    if kind=="trace":
                        trace.write(value)
                    if time.monotonic()-last_mirror>=1:
                        self.mirror_status()
                        last_mirror=time.monotonic()
        except OSError as error:
            self.fail_durable(error)

    def close(self) -> None:
        """Flush and stop both writers, joining each for up to 3 s wall-clock.

        Raises RuntimeError if a writer is still running or either one recorded an error.
        """
        with self.condition:
            self.closing=True
            self.condition.notify_all()
        if self.durable_thread.is_alive():
            self.queue.put(("close",None),timeout=1)
        self.local_thread.join(timeout=3)
        self.durable_thread.join(timeout=3)
        if self.local_thread.is_alive() or self.durable_thread.is_alive():
            raise RuntimeError("Navigation status/evidence flush exceeded its deadline")
        if self.local_error or self.durable_error:
            raise RuntimeError(f"Navigation status/evidence error: {self.local_error or self.durable_error}")
