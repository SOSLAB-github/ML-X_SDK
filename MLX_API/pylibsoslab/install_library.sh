#!/bin/bash
echo "move files from ./lib to /usr/lib/x86_64-linux-gnu"
sudo cp ./lib/libboost_python3-py36.so.1.65.1 /usr/lib/x86_64-linux-gnu
sudo cp ./lib/libpython3.6m.so.1.0 /usr/lib/x86_64-linux-gnu
