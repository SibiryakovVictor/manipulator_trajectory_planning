@echo off
echo "##################### PVS Studio Analysis: #####################"

rem Codepage 65001 or 866 breaks PVS
rem But codepage 20127 works

chcp 20127 >NUL 2>NUL

rem delete previous analysis result or it can trick us in thinking everything works
del "lst\pvs.plog"  >NUL 2>NUL
rem analyze
%PVS_STUDIO_PATH%\CLMonitor.exe analyzeFromDump -d "lst\pvs_dump.zip" -l "lst\pvs.plog"  -t "%~dp0\pvs_settings.xml" -c "%~dp0\ignore_warnings.pvsconfig" >NUL
rem assuming non-zero exit code mean some problem
if %ERRORLEVEL% GEQ 1 echo "ERROR: Couldn't perform analysis!" && exit 1

IF NOT EXIST lst\pvs.plog echo "No problems found" && exit 0

rem remove previous result
del "lst\pvs.plog.txt"  >NUL 2>NUL
rem if some problems were found - print them
%PVS_STUDIO_PATH%\PlogConverter.exe "lst\pvs.plog" --renderTypes=Txt -o lst >NUL
more lst\pvs.plog.txt
