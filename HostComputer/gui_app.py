import sys

from PySide6.QtCore import QMetaObject, Qt, QThread
from PySide6.QtWidgets import QApplication

from .main_window import HostMainWindow
from .serial_worker import SerialWorker


def main() -> int:
    app = QApplication(sys.argv)
    worker_thread = QThread()
    serial_worker = SerialWorker()
    serial_worker.moveToThread(worker_thread)

    window = HostMainWindow()
    window.set_serial_worker(serial_worker)

    worker_thread.started.connect(serial_worker.start)
    worker_thread.start()
    window.show()

    exit_code = app.exec()

    QMetaObject.invokeMethod(serial_worker, "stop", Qt.ConnectionType.BlockingQueuedConnection)
    worker_thread.quit()
    worker_thread.wait(2000)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
