@echo off
setlocal enabledelayedexpansion

call C:\emsdk\emsdk_env.bat

if not exist "%~dp0build-web" mkdir "%~dp0build-web"

:: Collect all .cpp files
set "SRC_FILES="
for %%f in ("%~dp0*.cpp") do (
    set "SRC_FILES=!SRC_FILES! "%%~f""
)

if "!SRC_FILES!"=="" (
    echo Error: No .cpp files found in "%~dp0"
    exit /b 1
)

:: Collect all .m3d files in the root folder and map them to virtual root (@/)
set "ASSET_FILES="
for %%a in ("%~dp0*.m3d") do (
    set "ASSET_FILES=!ASSET_FILES! --embed-file "%%~fa@/%%~nxa""
)

"C:\emsdk\upstream\emscripten\em++.exe" -std=c++23 -stdlib=libc++ -O3 -flto -msimd128 -fno-math-errno !SRC_FILES! -o "%~dp0build-web\index.html" ^
  -I"C:\raylib\src" ^
  -I"%~dp0include" ^
  "C:\raylib\build-web\raylib\libraylib.a" ^
  -sUSE_GLFW=3 ^
  -sALLOW_MEMORY_GROWTH=1 ^
  -sSINGLE_FILE=1 ^
  -DPLATFORM_WEB ^
  !ASSET_FILES! ^
  --shell-file "C:\raylib\src\minshell.html"
