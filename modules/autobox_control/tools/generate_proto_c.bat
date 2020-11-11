@echo off

Set PROTOC_C_EXE="C:/msys64/mingw64/bin/protoc.exe"

Set DIR_GEN="../../protoc_genfiles"
If NOT EXIST %DIR_GEN% (
  MKDIR %DIR_GEN%
)

for /R ".." %%f in (*.proto) do (
  For %%A in ("%filename%") do (
    Set Folder=%%~dpA
    Set Name=%%~nxA
  )
  %PROTOC_C_EXE% --c_out=DIR_GEN --proto_path=%Folder% %Name%
  
  REM echo "%%f"
)
