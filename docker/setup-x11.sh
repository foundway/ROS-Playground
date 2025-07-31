#!/bin/bash

# Setup X11 forwarding for macOS Docker containers
# Run this script before starting your Docker containers

echo "Setting up X11 forwarding for macOS..."

# Check if XQuartz is running
if ! pgrep -x "Xquartz" > /dev/null; then
    echo "Starting XQuartz..."
    open -a XQuartz
    sleep 3
fi

# Get the IP address for X11 forwarding
IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
echo "Your IP address is: $IP"

# Allow X11 connections from Docker containers
xhost + localhost
xhost + 127.0.0.1
xhost + $IP

echo "X11 forwarding setup complete!"
echo "You can now run GUI applications in your Docker container."
echo ""
echo "To test, run: docker exec -it jazzy rviz2"

echo "X11 forwarding setup complete!"
echo "You can now run GUI applications in your Docker container."
echo ""
echo "To test, run: docker exec -it humble rviz2" 