__all__ = ["HostMainWindow"]


def __getattr__(name):
    if name == "HostMainWindow":
        from .main_window import HostMainWindow

        return HostMainWindow
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
