:: copied from rviz_ogre_vendor/env_hook/rviz_ogre_vendor_library_path.bat
@echo off

call:ament_prepend_unique_value PATH "%AMENT_CURRENT_PREFIX%\opt\rviz_ogre_vendor\bin"

goto:eof

:: Prepend non-duplicate values to environment variables
:: using semicolons as separators and avoiding trailing separators.
:: first argument: the name of the result variable
:: second argument: the value
:ament_prepend_unique_value
  setlocal enabledelayedexpansion
  :: arguments
  set "listname=%~1"
  set "value=%~2"
  :: skip if path doesn't exist
  if NOT EXIST "%value%" (
    goto:eof
  )
  :: expand the list variable
  set "list=!%listname%!"
  :: check if the list contains the value
  set "is_duplicate="
  if "%list%" NEQ "" (
    for %%v in ("%list:;=";"%") do (
      if "%%~v" == "%value%" set "is_duplicate=1"
    )
  )
  :: if it is not a duplicate prepend it
  if "%is_duplicate%" == "" (
    :: if not empty, prepend a semi-colon
    if "!list!" NEQ "" set "list=;!list!"
    :: prepend the value
    set "list=%value%!list!"
  )
  endlocal & (
    :: set result variable in parent scope
    set "%~1=%list%"
  )
goto:eof
