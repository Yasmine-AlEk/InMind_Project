#!/bin/bash
set -e

echo "Installing .deb packages..."

cd /tmp/debs

sudo dpkg -i ros-humble-robotnik-controllers_*.deb

# Clean up after install
rm -rf /tmp/debs

echo ".deb package installed and cleaned up."
