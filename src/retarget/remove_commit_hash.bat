@echo off
chcp 65001 >NUL || exit /b

rem Этот батник нужен просто потому, что Кейл не умеет запускать .sh напрямую
rem >NUL 2>NUL это затыкание вывода, чтобы не засорять лог билда
rem если у вас несколько башей, установите переменную окружения %GIT_BASH_PATH% и не забудьте \ на конце

"%GIT_BASH_PATH%bash.exe" --login "%~dp0\remove_commit_hash.sh" >NUL || goto :error

rem Это костыль, текущему пользователю нужен полный доступ к папке с файлом retarget
rem Без этого sed почему-то делает файл доступным только для чтения
attrib -R "%~dp0\retarget.cpp"  >NUL || exit /b

goto :EOF

rem Произошла ошибка - выводим текст и падаем
:error
    echo --------------------
    echo "Error has occured when running update_commit_hash.bat!"
    echo "Please check environment variable GIT_BASH_PATH; it should contain path to bash with \ on the end"
    echo --------------------
    exit 1

