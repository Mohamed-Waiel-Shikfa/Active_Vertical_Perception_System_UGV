# SPDX-License-Identifier: GPL-3.0-only
"""Run a self_driving_api script once the simulation is ready, then leave the simulation running.

Started in the background by docker/start.sh when run.sh is given --script FILE."""
import runpy
import sys
import time
import traceback
from pathlib import Path

API_DIR = Path(__file__).resolve().parent


def banner(lines: list[str]) -> None:
    """Print `lines` between two rules of '=' sized to the longest line."""
    width = max(len(line) for line in lines) + 4
    print("\n" + "=" * width, *("  " + line for line in lines), "=" * width + "\n", sep="\n", flush=True)


def main() -> int:
    """Run sys.argv[1] as __main__ once the simulation is ready and report how it ended.

    Returns 2 on a usage error, otherwise 0 even if the script failed, so the simulation stays
    open; the outcome and wall/simulated durations (seconds) are printed in the closing banner."""
    if len(sys.argv) != 2:
        print("usage: run_script.py SCRIPT.py", file=sys.stderr)
        return 2
    script = Path(sys.argv[1]).resolve()
    # The image's API wins over a copy next to the script, so a script always matches its image.
    sys.path[:0] = [str(API_DIR), str(script.parent)]
    import self_driving_api as api

    banner([f"self_driving: {script.name} will start once the simulation is ready"])
    started_wall, started_sim = time.monotonic(), None
    try:
        api.wait_ready()
        started_sim = api._sim().sim_time()
        print(f"[self_driving] simulation ready after {time.monotonic() - started_wall:.0f} s; "
              f"running {script.name}", flush=True)
        started_wall = time.monotonic()
        sys.argv = [str(script)]
        runpy.run_path(str(script), run_name="__main__")
        outcome = "finished successfully"
    except SystemExit as exit_request:
        outcome = "finished successfully" if exit_request.code in (None, 0) else f"exited with {exit_request.code}"
    except BaseException as error:  # report anything (incl. Ctrl+C in the script) and keep the sim open
        traceback.print_exc()
        outcome = f"FAILED: {type(error).__name__}: {error}"
    wall = time.monotonic() - started_wall
    simulated = "" if started_sim is None else f", {api._sim().sim_time() - started_sim:.1f} s simulated"
    banner([f"self_driving: {script.name} {outcome}",
            f"({int(wall // 60)} min {wall % 60:.0f} s wall{simulated}).",
            "The simulation is still running; Ctrl+C in this terminal stops it."])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
