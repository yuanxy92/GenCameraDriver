dir="."
if [ $# -eq 0 ];then
    echo "No arguments supplied, will use [where I am] dir"
else
    dir=$1
fi

for i in `ls $dir/*.h265`
do
    #echo $i
    filename=${i##*/}
    #echo $filename
    #echo ${filename%.*}
    ffmpeg -y -i $i -vcodec copy $dir/${filename%.*}.mp4
done

#echo $dir