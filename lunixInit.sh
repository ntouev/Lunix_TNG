#!/bin/bash

rmmod ./lunix.ko
insmod ./lunix.ko
./lunix_dev_nodes.sh
./lunix-attach ttyS0
