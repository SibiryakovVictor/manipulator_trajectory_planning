@echo off
chcp 65001 >NUL || exit /b

rem ���� ������ ����� ������ ������, ��� ���� �� ����� ��������� .sh ��������
rem >NUL 2>NUL ��� ��������� ������, ����� �� �������� ��� �����
rem ���� � ��� ��������� �����, ���������� ���������� ��������� %GIT_BASH_PATH% � �� �������� \ �� �����

"%GIT_BASH_PATH%bash.exe" --login "%~dp0\remove_commit_hash.sh" >NUL || goto :error

rem ��� �������, �������� ������������ ����� ������ ������ � ����� � ������ retarget
rem ��� ����� sed ������-�� ������ ���� ��������� ������ ��� ������
attrib -R "%~dp0\retarget.cpp"  >NUL || exit /b

goto :EOF

rem ��������� ������ - ������� ����� � ������
:error
    echo --------------------
    echo "Error has occured when running update_commit_hash.bat!"
    echo "Please check environment variable GIT_BASH_PATH; it should contain path to bash with \ on the end"
    echo --------------------
    exit 1

