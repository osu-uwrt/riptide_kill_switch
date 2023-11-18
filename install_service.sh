#!/bin/bash

if [[ "$USER" == "root" ]]; then
    echo "This script must not be run as root"
    exit 1
fi

if

cp ./robostop.service "$HOME/.config/systemd/user/"
systemctl --user daemon-reload
systemctl --user enable robostop.service
systemctl --user start robostop.service
sudo loginctl enable-linger $USER
