@echo off

chcp 65001 >NUL || exit /b

rem Ётот батник нужен просто потому, что  ейл не умеет запускать .sh напр€мую
rem >NUL 2>NUL это затыкание вывода, чтобы не засор€ть лог билда
rem если у вас несколько башей, установите переменную окружени€ %GIT_BASH_PATH% и не забудьте \ на конце

rem --login помогает башу найти свое окружение
"%GIT_BASH_PATH%bash.exe" --login "%~dp0\update_commit_hash.sh" >NUL || goto :error

REM Ёто костыль, текущему пользователю нужен полный доступ к папке с файлом retarget
attrib -R "%~dp0\retarget.cpp" >NUL || exit /b

goto :EOF

rem ѕроизошла ошибка - выводим текст и падаем
:error
    echo --------------------
    echo "Error has occured when running update_commit_hash.bat!"
    echo "Please check environment variable GIT_BASH_PATH; it should contain path to bash with \ on the end"
    echo --------------------
    exit 1

