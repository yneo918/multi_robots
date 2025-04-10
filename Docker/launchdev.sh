# Build the Docker container
echo "🐳 Building the Docker image..."
docker compose build

# Run the Docker container (default command from docker-compose.yml)
echo "🚀 Running the ROS 2 project in Docker..."
docker-compose run multi_robots bash
echo "Connect to container in VSCode"