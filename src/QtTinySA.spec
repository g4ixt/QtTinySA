# -*- mode: python ; coding: utf-8 -*-

import sys

block_cipher = None

common = {
    'pathex': [],
    'binaries': [],
    'datas': [],
    'hiddenimports': [],
    'hookspath': [],
    'hooksconfig': {},
    'runtime_hooks': [],
    'excludes': ['pandas', 'setuptools', 'tk', 'wheel', 'zipp', 'pyyaml', 'packaging', 'altgraph', 'mkl', 'fortran', 'matlab'],
    'win_no_prefer_redirects': False,
    'win_private_assemblies': False,
    'cipher': block_cipher,
    'noarchive': False,
}

a = Analysis( ['QtTinySA.py'], **common)
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

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
    
# macOS
else:
    exe = EXE(
        pyz,
        a.scripts,
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
        exclude_binaries=True,
        icon='tinySA.icns'
    )

    app = BUNDLE(
        exe,
        a.binaries,
        a.datas,
        strip=False,
        upx=True,
        upx_exclude=[],
        name='QtTinySA.app',
        icon='tinySA.icns',
        bundle_identifier=None,
        info_plist={
            'CFBundleName': 'QtTinySA',
            'CFBundleDisplayName': 'QtTinySA',
            'NSHumanReadableCopyright': 'Copyright © 2025 QtTinySA',
            'LSUIElement': False,
            'LSBackgroundOnly': False,
        },
    )
