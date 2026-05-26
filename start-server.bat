@echo off
echo Starting RoboChickens server on port 3000...
echo Local:  http://localhost:3000
echo Public: requires start-tunnel.bat with a Cloudflare tunnel
echo.
cd /d "%~dp0"
node dist/world/world-server.js
