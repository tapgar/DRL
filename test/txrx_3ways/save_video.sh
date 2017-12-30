ffmpeg -f rawvideo -pixel_format rgb24 -video_size 1200x900 -framerate 60  -r $2 -i out/temp.out -vf "vflip" out/$1.mp4
