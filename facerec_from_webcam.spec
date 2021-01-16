# -*- mode: python ; coding: utf-8 -*-

block_cipher = None

added_files = [
        ('C:\\School\\2020-21\\ENEE408I\\Sample Code\\CompanionRobot\\cmsc426env\\Lib\\site-packages\\face_recognition_models\\models\\*.dat',
        'face_recognition_models/models'),
        ('Nathan.jpg', '.'),
        ('Renee.jpeg', '.')]
a = Analysis(['facerec_from_webcam.py'],
             pathex=['C:\\Users\\thequ\\Documents\\GitHub\\Companion_Robot'],
             binaries=[],
             datas=added_files,
             hiddenimports=[],
             hookspath=[],
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher,
             noarchive=False)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)
exe = EXE(pyz,
          a.scripts,
          [],
          exclude_binaries=True,
          name='facerec_from_webcam',
          debug=False,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          console=True )
coll = COLLECT(exe,
               a.binaries,
               a.zipfiles,
               a.datas,
               strip=False,
               upx=True,
               upx_exclude=[],
               name='facerec_from_webcam')
