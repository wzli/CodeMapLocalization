#/bin/sh
out_file=${2:-frames.raw}
ffmpeg -i ${1} -f rawvideo $out_file
