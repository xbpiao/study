﻿rem 提交svn前需要清除的无效数据，这里先写批处理吧，mac和linux下的有机会再补

rem 清除proj.android
rmdir /q /s proj.android\libs
rmdir /q /s proj.android\bin
rmdir /q /s proj.android\obj
rmdir /q /s proj.android\assets
rmdir /q /s proj.android-studio\build
rmdir /q /s bin

rem 清除proj.win32
rmdir /q /s proj.win32\Debug.win32
rmdir /q /s proj.win32\.vs
del /q /s *.bak
del /q /s *.so
del /q /s *.a
del /q /s *.sdf
del /q /s *.VC.db
del /q /s *.VC.opendb
del /q /s /ah proj.win32\*.suo