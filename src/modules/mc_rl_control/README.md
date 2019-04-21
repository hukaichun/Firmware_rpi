---
title: 'mc_rl_control'
disqus: hackmd
---

mc_rl_control
===

## Table of Contents

[TOC]

## Branches
1. **master**: merge from socket_log_data. Using TCP socket publishing states and action pare for training. 
2. **socket_log_data**: same as above.
3. **log_states**: log data to raspberry pi and need to fetch log files for training.

## Concept
firmwate will be created as a exectuable program named px4 loacate at 
```gherkin=
~/
```
the config file is a startup script for loading modules in filght program **pxh**.

## Usage

start RL controller by start test.config file
```gherkin=
sudo ./bin/px4 -s test.config
```

Sub-Folders
---
1. **decode_api_py**: 
    1. **decode_file.py**: using for decoding log files fetch from raspberry pi and is used to be with log_states git branch.
    2. **tcp_client.py**: receive logs using tcp protocol and is used to be with socket_log_data git branch.
2. **tools**:
    1. **tf2cpp.py**: load tensorflow model veriables and generate cpp file.
    2. **makefile**: it is easy to understand so please help yourself.

