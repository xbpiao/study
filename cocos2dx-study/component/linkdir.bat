@echo off
setlocal enabledelayedexpansion
rem Save the current path
set currentPath=%~dp0
rem Save the current drive letter
set currentDriver=%~d0
rem Directly use the relative path of the batch file
%currentDriver%
cd "%currentPath%"

echo %currentPath%

rmdir /S /Q "%currentPath%cocos2d-x-3.15"

mklink /J "%currentPath%cocos2d-x-3.15" "%currentPath%..\..\..\xbpiao-cocos2d-x"


@echo on