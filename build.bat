@echo off
cd /d "%~dp0"
cargo build --release -p viewer || exit /b 1
copy /Y target\release\autorouter.exe . >nul
echo Done: autorouter.exe (GUI); autorouter.exe -nogui board.dsn (CLI)
