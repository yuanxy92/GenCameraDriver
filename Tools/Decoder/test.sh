for i in `ls *.bin`
do
    #echo $i
    filename=${i%.*}
    echo $filename
    ./Decoder $i "$filename.avi"
done
