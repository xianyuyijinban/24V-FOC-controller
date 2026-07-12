"""
FOC Device Bridge — System Tray Application

Owns the COM port exclusively. Injects wheel events into the desktop
via Win32 SendInput. HostComputer GUI can connect as a spectator
through the named-pipe IPC server.
"""

import sys

from PySide6.QtCore import QTimer
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QApplication,
    QInputDialog,
    QMenu,
    QMessageBox,
    QSystemTrayIcon,
)

import serial.tools.list_ports

from .config_manager import BridgeConfig
from .wheel_injector import Win32WheelInjector
from .serial_owner import SerialOwner
from .session_manager import SessionManager
from .session_manager import (
    STATE_DISCONNECTED,
    STATE_CONNECTED_IDLE,
    STATE_WHEEL_ACTIVE,
    STATE_WHEEL_ENABLING,
    STATE_WHEEL_DISABLING,
)
from .ipc_server import IpcServer


# ── Tray tooltip states ────────────────────────────────────────────
STATE_TOOLTIPS = {
    STATE_DISCONNECTED:    "FOC Bridge — Disconnected",
    STATE_CONNECTED_IDLE:  "FOC Bridge — Connected (wheel idle)",
    STATE_WHEEL_ENABLING:  "FOC Bridge — Enabling wheel...",
    STATE_WHEEL_ACTIVE:    "FOC Bridge — Wheel Active 🖱️",
    STATE_WHEEL_DISABLING: "FOC Bridge — Disabling wheel...",
}


class BridgeApp:
    """Main system-tray application."""

    def __init__(self, qapp: QApplication):
        self._app = qapp
        self._config = BridgeConfig.load()

        # ── Components ────────────────────────────────────────
        self._injector = Win32WheelInjector(reverse=self._config.reverse_direction)

        # SerialOwner runs on the main thread with QTimer-based polling.
        # QTimer + 10ms interval is non-blocking and avoids cross-thread complexity.
        self._serial_owner = SerialOwner(self._config)

        self._session_mgr = SessionManager(self._config)
        self._ipc_server = IpcServer()

        # ── Wire signals ──────────────────────────────────────
        self._wire_signals()

        # ── Tray icon ─────────────────────────────────────────
        self._tray = QSystemTrayIcon()
        self._tray.setToolTip(STATE_TOOLTIPS[STATE_DISCONNECTED])
        # Use a built-in icon (no custom .ico required for MVP)
        self._tray.setIcon(self._app.style().standardIcon(
            self._app.style().StandardPixmap.SP_ComputerIcon
        ))
        self._build_menu()
        self._tray.show()

        # ── Start IPC server ──────────────────────────────────
        self.startup_ok = self._ipc_server.start()
        if not self.startup_ok:
            self._tray.showMessage(
                "FOC Device Bridge",
                "Bridge 已在系统托盘运行，请勿重复启动。",
                QSystemTrayIcon.MessageIcon.Information,
                3500,
            )
            QTimer.singleShot(0, self._app.quit)
            return

        self._tray.showMessage(
            "FOC Device Bridge",
            "Bridge 已启动并驻留系统托盘。上位机主界面请运行 24V_FOC_Host.exe。",
            QSystemTrayIcon.MessageIcon.Information,
            4500,
        )

        # ── Auto-connect if last port is available ────────────
        if self._config.last_port:
            available = [p.device for p in serial.tools.list_ports.comports()]
            if self._config.last_port in available:
                self._log("INFO", f"Auto-connecting to {self._config.last_port}...")
                self._serial_owner.connect_port(self._config.last_port, self._config.last_baud)

    # ── Signal Wiring ─────────────────────────────────────────────

    def _wire_signals(self):
        so = self._serial_owner
        sm = self._session_mgr
        ipc = self._ipc_server

        # Serial → SessionManager
        so.ack_received.connect(sm.on_ack)

        # Serial → IPC broadcast
        so.serial_rx.connect(ipc.broadcast_serial_rx)
        so.connection_changed.connect(self._on_connection_changed)
        so.log_line.connect(self._log)

        # SessionManager → Serial (commands)
        sm.command_to_send.connect(self._on_send_command)

        # SessionManager → IPC
        sm.state_changed.connect(self._on_state_changed)
        sm.error_occurred.connect(self._on_error)

        # Wheel events → injector (on main thread via signal)
        so.wheel_event.connect(self._on_wheel_event)
        so.wheel_status_update.connect(ipc.broadcast_wheel_status)

        # IPC → Serial (GUI spectator TX)
        ipc.serial_tx.connect(self._on_ipc_serial_tx)
        ipc.connect_request.connect(self._on_ipc_connect_request)
        ipc.wheel_enable_request.connect(self._on_ipc_wheel_enable)
        ipc.wheel_config_request.connect(self._on_ipc_wheel_config)

    # ── Tray Menu ──────────────────────────────────────────────────

    def _build_menu(self):
        menu = QMenu()

        # Wheel control
        self._action_enable = QAction("⚙ Enable Wheel", menu)
        self._action_enable.setCheckable(True)
        self._action_enable.triggered.connect(self._on_toggle_wheel)
        menu.addAction(self._action_enable)

        self._action_reverse = QAction("🔄 Reverse Direction", menu)
        self._action_reverse.setCheckable(True)
        self._action_reverse.setChecked(self._config.reverse_direction)
        self._action_reverse.triggered.connect(self._on_toggle_reverse)
        menu.addAction(self._action_reverse)

        menu.addSeparator()

        # Stats
        action_stats = QAction("📊 Show Stats", menu)
        action_stats.triggered.connect(self._show_stats)
        menu.addAction(action_stats)

        menu.addSeparator()

        # Serial
        action_ports = QAction("📡 Select COM Port...", menu)
        action_ports.triggered.connect(self._select_com_port)
        menu.addAction(action_ports)

        self._action_connect = QAction("🔌 Connect", menu)
        self._action_connect.triggered.connect(self._on_connect)
        menu.addAction(self._action_connect)

        self._action_disconnect = QAction("⏏ Disconnect", menu)
        self._action_disconnect.triggered.connect(self._on_disconnect)
        self._action_disconnect.setVisible(False)
        menu.addAction(self._action_disconnect)

        menu.addSeparator()

        # Quit
        action_quit = QAction("❌ Quit", menu)
        action_quit.triggered.connect(self._on_quit)
        menu.addAction(action_quit)

        self._tray.setContextMenu(menu)
        self._menu = menu

    # ── Event Handlers ──────────────────────────────────────────────

    def _on_wheel_event(self, event):
        """Marshalled to main thread via Qt signal. Safe to call SendInput here."""
        if self._session_mgr.current_state() != STATE_WHEEL_ACTIVE:
            return
        if not event.session_active:
            self._on_error("Dropped wheel event without an active firmware session")
            return
        self._injector.inject(event.delta_steps)

    def _on_send_command(self, text: str):
        """SessionManager wants to send a command. Append newline, write to serial."""
        self._serial_owner.send_text(text)
        self._log("TX", text)

    def _on_connection_changed(self, connected: bool):
        self._session_mgr.set_connected(connected)
        self._action_connect.setVisible(not connected)
        self._action_disconnect.setVisible(connected)
        state = self._session_mgr.current_state()
        port = self._serial_owner._port_name if connected else ""
        baud = self._serial_owner._baud_rate if connected else 0
        self._ipc_server.broadcast_conn_state(state, port, baud)

        if connected:
            self._config.last_port = self._serial_owner._port_name
            self._config.last_baud = self._serial_owner._baud_rate
            self._config.save()

    def _on_state_changed(self, state: str):
        self._tray.setToolTip(STATE_TOOLTIPS.get(state, f"FOC Bridge — {state}"))
        self._action_enable.setChecked(state == STATE_WHEEL_ACTIVE)

        if state in (STATE_WHEEL_ACTIVE, STATE_CONNECTED_IDLE, STATE_WHEEL_ENABLING, STATE_WHEEL_DISABLING):
            port = self._serial_owner._port_name
            baud = self._serial_owner._baud_rate
            self._ipc_server.broadcast_conn_state(state, port, baud)

        if state in (STATE_WHEEL_ACTIVE, STATE_CONNECTED_IDLE):
            self._ipc_server.broadcast_wheel_status(
                self._serial_owner.wheel_status_snapshot(
                    session_active=(state == STATE_WHEEL_ACTIVE)
                )
            )

    def _on_error(self, message: str):
        self._log("ERROR", message)
        self._ipc_server.broadcast_error(message)

    def _on_ipc_serial_tx(self, data: bytes):
        """GUI spectator wants to send data to the device."""
        self._serial_owner.send_bytes(data)

    def _on_ipc_connect_request(self, port: str, baud: int):
        """Let the visible Host GUI select the COM port owned by Bridge."""
        self._config.last_port = port
        self._config.last_baud = baud
        self._config.save()
        self._serial_owner.connect_port(port, baud)

    def _on_ipc_wheel_enable(self, enable: bool):
        if enable:
            self._session_mgr.start_enable()
        else:
            self._session_mgr.start_disable()

    def _on_ipc_wheel_config(self, cfg: dict):
        self._config.apply_wheel_config_dict(cfg)
        self._config.save()
        # If wheel is active, re-send WHEEL:CFG to device
        if self._session_mgr.current_state() == STATE_WHEEL_ACTIVE:
            w = self._config.wheel_config_dict()
            cmd = f"WHEEL:CFG,{w['count']},{w['strength']:.3f},{w['width']:.3f},{w['damping']:.3f},{w['limit']:.3f}"
            self._serial_owner.send_text(cmd)

    # ── Tray Actions ────────────────────────────────────────────────

    def _on_toggle_wheel(self, checked: bool):
        if checked:
            self._session_mgr.start_enable()
        else:
            self._session_mgr.start_disable()

    def _on_toggle_reverse(self, checked: bool):
        self._config.reverse_direction = checked
        self._injector.set_reverse(checked)
        self._config.save()

    def _show_stats(self):
        sm = self._session_mgr
        so = self._serial_owner
        state = sm.current_state()
        msg = (
            f"State: {state}\n"
            f"Port: {so._port_name} @ {so._baud_rate}\n"
            f"Connected: {so.is_connected()}\n"
            f"Wheel Position: {so._wheel_position} steps\n"
            f"Total Delta: {so._wheel_total_delta}\n"
            f"Events Received: {so._wheel_events_received}\n"
            f"Reverse: {self._config.reverse_direction}\n"
            f"\nWheel Config:\n"
            f"  Count: {self._config.wheel_config_dict()['count']}\n"
            f"  Strength: {self._config.wheel_strength:.3f}\n"
            f"  Width: {self._config.wheel_width:.3f}\n"
            f"  Damping: {self._config.wheel_damping:.3f}\n"
            f"  Limit: {self._config.wheel_limit:.3f}"
        )
        QMessageBox.information(None, "FOC Bridge Stats", msg)

    def _select_com_port(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            self._log("ERROR", "No COM ports available")
            return
        port, ok = QInputDialog.getItem(None, "Select COM Port", "Port:", ports, 0, False)
        if ok and port:
            self._config.last_port = port
            self._config.save()
            self._serial_owner.connect_port(port, self._config.last_baud)

    def _on_connect(self):
        if self._config.last_port:
            self._serial_owner.connect_port(self._config.last_port, self._config.last_baud)
        else:
            self._select_com_port()

    def _on_disconnect(self):
        # Disable wheel first if active
        if self._session_mgr.current_state() == STATE_WHEEL_ACTIVE:
            self._session_mgr.start_disable()
        self._serial_owner.disconnect()

    def _on_quit(self):
        # Graceful shutdown: disable wheel, disconnect serial, stop IPC
        if self._session_mgr.current_state() == STATE_WHEEL_ACTIVE:
            self._session_mgr.stop()  # emergency stop
        self._serial_owner.disconnect()
        self._ipc_server.stop()

        self._config.save()
        self._app.quit()

    def _log(self, level: str, message: str):
        print(f"[{level}] {message}")


# ── Entry Point ─────────────────────────────────────────────────────

def main() -> int:
    # Must have QApplication before anything else
    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False)  # tray app

    bridge = BridgeApp(app)
    # Keep a reference so it doesn't get garbage-collected
    # (stored as attribute on QApplication for safety)
    app._bridge = bridge

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
