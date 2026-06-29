# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path

from PyInstaller.utils.hooks import collect_all


PROJECT_ROOT = Path(SPECPATH).parent
PYQTGRAPH_DATAS, PYQTGRAPH_BINARIES, PYQTGRAPH_HIDDENIMPORTS = collect_all("pyqtgraph")
SERIAL_DATAS, SERIAL_BINARIES, SERIAL_HIDDENIMPORTS = collect_all("serial")


a = Analysis(
    [str(PROJECT_ROOT / "HostComputer" / "host_gui_launcher.py")],
    pathex=[str(PROJECT_ROOT)],
    binaries=PYQTGRAPH_BINARIES + SERIAL_BINARIES,
    datas=PYQTGRAPH_DATAS + SERIAL_DATAS,
    hiddenimports=PYQTGRAPH_HIDDENIMPORTS + SERIAL_HIDDENIMPORTS,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='24V_FOC_Host',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name='24V_FOC_Host',
)
