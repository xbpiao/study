@echo off
setlocal enabledelayedexpansion
rem ���浱ǰ·��
set currentPath=%~dp0
rem ���浱ǰ�������̷�
set currentDriver=%~d0
rem ֱ��ʹ���������ļ������·��
%currentDriver%
cd "%currentPath%"

echo %currentPath%

rmdir /S /Q "%currentPath%cocos2d-x-3.15"

mklink /J "%currentPath%cocos2d-x-3.15" "%currentPath%..\..\..\xbpiao-cocos2d-x-3.15"


@echo on