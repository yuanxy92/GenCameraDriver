#!/usr/bin/expect
# argv1 host
# argv2 keyname
# argv3 local dir
scp -i $2 -r zhu-ty@$1:/home/zhu-ty/Projects/GenCameraDriver/build/saved $3
