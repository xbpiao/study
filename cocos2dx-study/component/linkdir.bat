@echo off
setlocal enabledelayedexpansion
rem 保存当前路径
set currentPath=%~dp0
rem 保存当前驱动器盘符
set currentDriver=%~d0
rem 直接使用批处理文件的相对路径
%currentDriver%
cd "%currentPath%"

echo %currentPath%

rmdir /S /Q "%currentPath%cocos2d-x-3.15"

mklink /J "%currentPath%cocos2d-x-3.15" "%currentPath%..\..\..\xbpiao-cocos2d-x-3.15"


@echo on