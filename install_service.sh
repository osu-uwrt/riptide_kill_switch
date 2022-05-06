#!/bin/bash
sudo cp ./robostop.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable robostop
sudo systemctl start robostop

