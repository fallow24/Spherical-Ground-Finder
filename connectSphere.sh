#!/bin/bash
ssh -Y -t deadalus@10.42.0.1 "sudo date --set \"$(date "+%d %b %Y %H:%M:%S")\"; tmux new-session \; split-window -h \; split-window -v \; attach; exec bash -I"
