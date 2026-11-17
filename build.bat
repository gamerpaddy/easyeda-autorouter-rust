@echo off
cd /d "%~dp0"
cargo build --release -p viewer || exit /b 1
echo Done: target\release\autorouter.exe (GUI, start it with autorouter.lnk); autorouter.exe -nogui board.dsn (CLI)
