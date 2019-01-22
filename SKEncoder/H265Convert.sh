dir="."
rate=30
if [ $# -eq 0 ];then
    echo "No arguments supplied, will use [where I am] dir"
else
    dir=$1
fi

if [ $# -gt 1 ];then
    rate=$2
fi

for i in `ls $dir/*.h265`
do
    #echo $i
    filename=${i##*/}
    #echo $filename
    #echo ${filename%.*}
    ffmpeg -y -r ${rate} -i $i -vcodec copy $dir/${filename%.*}.mp4
done

#echo $dir