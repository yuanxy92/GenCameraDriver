#!/usr/bin/expect
# argv1 host
# argv2 keyname
# argv3 server dir
# argv4 local dir
scp -i $2 -r zhu-ty@$1:/home/zhu-ty/Projects/shell/$3 $4
