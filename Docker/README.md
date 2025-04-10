# Docker
Contains file for running the porject in Docker container.

## Before Using
Make sure you have Docker Desktop and docker-compose installed and working on your machine

## How to Use
Run the appropriate launch script depending on your system and intention. 
"launchmain" - runs the entire project automatically
"launchdev" - creates dev container to use as development environment (WIP)

### Windows:
1. Navigate to Docker directory
2. Enable script execution if you haven't before: Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser 
3. run .ps1 file: .\launch{main/dev}.ps1

### Linux/Mac:
1. Navigate to Docker directory
2. Enable script execution if you haven't before: chmod +x launch{main/dev}.sh
3. run .sh file: ./launch{main/dev}.sh