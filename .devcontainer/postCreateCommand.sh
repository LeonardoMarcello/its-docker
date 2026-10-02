#!/bin/bash

# Install additional Python packages
pip3 install -r requirements.txt

# Run rosdep to install any missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Any other setup commands can be added here
echo "Post-create commands executed successfully."
