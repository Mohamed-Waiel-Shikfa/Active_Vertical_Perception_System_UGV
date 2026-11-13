"""Lightweight probe of the persistent, genuinely live observer in this container."""
import json
import sys
from urllib.error import HTTPError,URLError
from urllib.request import Request,urlopen


def main():
    try:
        try:
            with urlopen(Request("http://127.0.0.1:8765/health",data=b"{}",method="POST"),timeout=3) as response:
                state=json.load(response)
        except HTTPError as error:
            state=json.loads(error.read())
        if state.get("ok") is not True:
            raise RuntimeError(json.dumps(state))
        print(state["message"])
        return 0
    except (OSError,URLError,TimeoutError,ValueError,RuntimeError) as error:
        print(f"greenhouse health check failed: {error}",file=sys.stderr)
        return 1


if __name__=="__main__":
    raise SystemExit(main())
