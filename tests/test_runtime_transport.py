import importlib.util
import json
import unittest
from pathlib import Path
from unittest.mock import Mock,patch

from greenhouse import transport

spec=importlib.util.spec_from_file_location("apply_terrain_test",Path(__file__).resolve().parents[1]/"tools/apply_terrain.py")
apply=importlib.util.module_from_spec(spec)
spec.loader.exec_module(apply)


class RuntimeTransportTests(unittest.TestCase):
    def test_wait_status_is_independent_of_transaction_locals(self):
        state={"paused":True,"sim_time":12.0,"grounds":[]}
        with patch.object(apply,"world_status",return_value=state):
            self.assertIs(apply.wait_status(lambda s:s["paused"],timeout=.1),state)

    def test_serial_native_requests_reuse_one_node_and_check_replies(self):
        process=Mock()
        process.poll.return_value=None
        process.stdout.readline.side_effect=[
            json.dumps({"ok":True,"text":"data: true\n"})+"\n",
            json.dumps({"ok":True,"text":"data: true\n"})+"\n"]
        previous=transport._client
        transport._client=None
        try:
            with patch.object(transport.subprocess,"Popen",return_value=process) as create, \
                    patch.object(transport.select,"select",return_value=([process.stdout],[],[])), \
                    patch.object(transport,"trace_request") as trace:
                self.assertTrue(transport.service("/first"))
                self.assertTrue(transport.service("/second"))
                create.assert_called_once()
                self.assertEqual(process.stdin.write.call_count,2)
                self.assertNotEqual(trace.call_args_list[0].args[0]["id"],trace.call_args_list[1].args[0]["id"])
        finally:
            transport._client=previous
