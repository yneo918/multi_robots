# Run the Docker container (default command from docker-compose.yml)
echo "Running the ROS 2 project in Docker..."
docker-compose -f docker-compose.dev.yml up --build
#Start-Process powershell -ArgumentList "docker exec -it multi_robots-dev bash"
echo "Connect to container in VSCode to edit files."