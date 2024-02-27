# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['QtTinySA.py'],
    pathex=[],
    binaries=[],
    datas=[('QtTSAprefs.db', '.')],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['pandas','setuptools', 'tk', 'wheel', 'zipp', 'pyyaml', 'packaging', 'altgraph', 'mkl', 'fortran', 'matlab'],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='QtTinySA',
    icon=['tinySA.ico'],
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,
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
    upx=True,
    upx_exclude=[],
    name='QtTinySA',
)
