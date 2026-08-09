# -*- mode: python ; coding: utf-8 -*-

import sys

common = {
    'pathex': [],
    'binaries': [],
    'datas': [('QtTSAprefs.db', '.'), ('*.ui', '.'), ('./modules/10_baseline.txt', '.'), ('./modules/1152_baseline.txt', '.')],
    'hiddenimports': [],
    'hookspath': [],
    'hooksconfig': {},
    'runtime_hooks': [],
    'excludes': ['tkinter', 'pandas', 'setuptools', 'tk', 'wheel', 'zipp', 'pyyaml', 'packaging', 'altgraph', 'mkl', 'fortran', 'matlab'],
    'win_no_prefer_redirects': False,
    'win_private_assemblies': False,
    'noarchive': False,
    }

a = Analysis( ['QtTinySA.py'], **common)
pyz = PYZ(a.pure, a.zipped_data)

if sys.platform != 'darwin':
    exe = EXE(
        pyz,
        a.scripts,
        a.binaries,
        a.zipfiles,
        a.datas,
        [],
        name='QtTinySA',
        debug=False,
        bootloader_ignore_signals=False,
        strip=False,
        upx=True,
        upx_exclude=[],
        runtime_tmpdir=None,
        console=True,
        disable_windowed_traceback=False,
        argv_emulation=False,
        target_arch=None,
        codesign_identity=None,
        entitlements_file=None,
        icon='tinySA.ico'
        )

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='QtTinySA.module',
)
