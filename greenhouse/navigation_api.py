# SPDX-License-Identifier: GPL-3.0-only
import json
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def request(action: str, values: dict[str, Any] | None = None, timeout: float = 25) -> dict[str, Any]:
    """POST `values` as JSON to the local navigation coordinator at 127.0.0.1:8765/`action`.

    `timeout` is wall-clock seconds. Returns the reply object; raises RuntimeError if the coordinator
    is unreachable or the reply is not {"ok": true, ...}.
    """
    message = Request("http://127.0.0.1:8765/" + action,
                      data=json.dumps(values or {}).encode("utf-8"),
                      headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urlopen(message, timeout=timeout) as response:
            result = json.load(response)
    except HTTPError as error:
        result = json.loads(error.read().decode("utf-8"))
    except (URLError, TimeoutError) as error:
        raise RuntimeError(f"Navigation coordinator unavailable: {error}") from error
    if result.get("ok") is not True:
        raise RuntimeError(result.get("error", "Navigation request rejected"))
    return result
