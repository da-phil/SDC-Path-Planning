#!/bin/bash

palette="/tmp/palette.png"
filters="fps=5,scale=480:-1:flags=lanczos"
bitrate="2048k"

ffmpeg -v warning -i $1 -vf "$filters,palettegen" -y $palette
ffmpeg -v warning -i $1 -i $palette -lavfi "$filters [x]; [x][1:v] paletteuse" -y $2 