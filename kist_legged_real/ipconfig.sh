#!/bin/bash

sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

sudo ifconfig enx4865ee15a3f4 down
sudo ifconfig enx4865ee15a3f4 up 192.168.123.162 netmask 255.255.255.0

