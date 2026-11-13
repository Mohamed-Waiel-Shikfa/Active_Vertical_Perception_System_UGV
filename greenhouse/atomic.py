"""Atomic old-or-new file replacement for state files that other processes poll."""
from pathlib import Path


def replace(source: Path, destination: Path | str) -> None:
    """Rename ``source`` over ``destination`` in one step.

    A POSIX rename is atomic, so a reader sees either the old or the new file, never a partly
    written one. Both paths must be on the same filesystem; any ``OSError`` propagates.
    """
    source.replace(destination)
