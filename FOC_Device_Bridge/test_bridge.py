import json
import sys
import unittest
import uuid
from pathlib import Path

from PySide6.QtCore import QCoreApplication


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from FOC_Device_Bridge.bridge_app import BridgeApp
from FOC_Device_Bridge.config_manager import BridgeConfig
from FOC_Device_Bridge.ipc_protocol import (
    KIND_CONNECT_REQUEST,
    KIND_CONN_STATE,
    KIND_WHEEL_STATUS,
)
from FOC_Device_Bridge.ipc_server import IpcServer
from FOC_Device_Bridge.session_manager import (
    SessionManager,
    STATE_CONNECTED_IDLE,
    STATE_WHEEL_ACTIVE,
)
from HostComputer.data_parser import WheelEvent
from HostComputer.transport import BridgeTransport


class _FakeSessionManager:
    def __init__(self, state):
        self.state = state

    def current_state(self):
        return self.state


class _FakeInjector:
    def __init__(self):
        self.deltas = []

    def inject(self, delta):
        self.deltas.append(delta)


class _FakeTray:
    def setToolTip(self, _text):
        pass


class _FakeAction:
    def __init__(self):
        self.checked = False

    def setChecked(self, checked):
        self.checked = checked


class _FakeSerialOwner:
    _port_name = "COM7"
    _baud_rate = 1000000

    def wheel_status_snapshot(self, session_active):
        return {
            "position_steps": 0,
            "total_delta": 0,
            "events_received": 0,
            "session_active": session_active,
        }


class _FakeIpcServer:
    def __init__(self):
        self.conn_states = []
        self.wheel_statuses = []

    def broadcast_conn_state(self, state, port, baud):
        self.conn_states.append((state, port, baud))

    def broadcast_wheel_status(self, status):
        self.wheel_statuses.append(status)


class TestBridgeSession(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def test_enable_sequence_establishes_mode_before_session(self):
        manager = SessionManager(BridgeConfig())
        commands = []
        manager.command_to_send.connect(commands.append)
        manager.set_connected(True)
        self.assertEqual(manager.current_state(), STATE_CONNECTED_IDLE)

        manager.start_enable()
        acks = [
            "CUR_STREAM,OK,OFF",
            "APP_MODE,OK,SCROLL_WHEEL (ctrl_mode=2)",
            "WHEEL:SESSION,OK",
            "WHEEL:CFG,OK",
            "UNLOCK,OK,1",
            "ENABLE,OK,1",
        ]
        for ack in acks:
            self.assertTrue(manager.on_ack(ack))

        self.assertEqual(manager.current_state(), STATE_WHEEL_ACTIVE)
        self.assertEqual(commands[0], "TELEM:CUR,OFF")
        self.assertEqual(commands[1], "CMD:APP_MODE,SCROLL_WHEEL")
        self.assertTrue(commands[2].startswith("WHEEL:SESSION,"))
        manager.stop()

    def test_bridge_injects_only_active_session_events(self):
        bridge = BridgeApp.__new__(BridgeApp)
        bridge._session_mgr = _FakeSessionManager(STATE_CONNECTED_IDLE)
        bridge._injector = _FakeInjector()
        errors = []
        bridge._on_error = errors.append

        active_event = WheelEvent(delta_steps=1, flags=0x0008)
        bridge._on_wheel_event(active_event)
        self.assertEqual(bridge._injector.deltas, [])

        bridge._session_mgr.state = STATE_WHEEL_ACTIVE
        bridge._on_wheel_event(WheelEvent(delta_steps=2, flags=0))
        self.assertEqual(bridge._injector.deltas, [])
        self.assertEqual(len(errors), 1)

        bridge._on_wheel_event(WheelEvent(delta_steps=-3, flags=0x0008))
        self.assertEqual(bridge._injector.deltas, [-3])

    def test_wheel_active_state_broadcasts_initial_zero_status(self):
        bridge = BridgeApp.__new__(BridgeApp)
        bridge._tray = _FakeTray()
        bridge._action_enable = _FakeAction()
        bridge._serial_owner = _FakeSerialOwner()
        bridge._ipc_server = _FakeIpcServer()

        bridge._on_state_changed(STATE_WHEEL_ACTIVE)

        self.assertTrue(bridge._action_enable.checked)
        self.assertEqual(
            bridge._ipc_server.conn_states,
            [(STATE_WHEEL_ACTIVE, "COM7", 1000000)],
        )
        self.assertEqual(bridge._ipc_server.wheel_statuses[-1]["position_steps"], 0)
        self.assertTrue(bridge._ipc_server.wheel_statuses[-1]["session_active"])


class TestBridgeIpcState(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def test_transport_decodes_connection_and_wheel_status(self):
        transport = BridgeTransport()
        states = []
        statuses = []
        transport.connection_state_changed.connect(states.append)
        transport.wheel_status_changed.connect(statuses.append)

        conn = {"state": "CONNECTED_IDLE", "port": "COM9", "baud": 1000000}
        transport._handle_control_frame(KIND_CONN_STATE, json.dumps(conn).encode())
        transport._handle_control_frame(
            KIND_WHEEL_STATUS,
            json.dumps({"position_steps": 12, "total_delta": 30}).encode(),
        )

        self.assertTrue(transport.device_connected())
        self.assertEqual(states[-1]["port"], "COM9")
        self.assertEqual(statuses[-1]["position_steps"], 12)

    def test_ipc_server_caches_latest_state_for_new_clients(self):
        server = IpcServer()
        server.broadcast_conn_state("CONNECTED_IDLE", "COM9", 1000000)
        cached = json.loads(server._last_conn_state.decode("utf-8"))
        self.assertEqual(cached["state"], "CONNECTED_IDLE")
        self.assertEqual(cached["port"], "COM9")

    def test_ipc_server_rejects_a_second_live_owner(self):
        server_name = f"FOC_Device_Bridge_test_{uuid.uuid4().hex}"
        first = IpcServer(server_name)
        second = IpcServer(server_name)
        try:
            self.assertTrue(first.start())
            self.assertFalse(second.start())
        finally:
            second.stop()
            first.stop()

    def test_ipc_server_dispatches_host_connect_request(self):
        server = IpcServer()
        requests = []
        server.connect_request.connect(lambda port, baud: requests.append((port, baud)))

        server._dispatch(
            KIND_CONNECT_REQUEST,
            json.dumps({"port": "COM7", "baud": 1000000}).encode("utf-8"),
        )

        self.assertEqual(requests, [("COM7", 1000000)])


if __name__ == "__main__":
    unittest.main()
