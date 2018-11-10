#keep this file outside of Teamcode folder

src_folder="src/main/java/org/firstinspires/ftc/teamcode/Tempest"
des_folder="src/org/firstinspires/ftc/teamcode/Tempest"

for file in ${src_folder}/*
do
    echo $(basename $file)
    file_only=$(basename $file)    
    des="http://192.168.49.1:8080/java/file/save?f=/${des_folder}/${file_only}"
    src="data@${src_folder}/${file_only}"
    echo $des
    echo $src
    echo "connecting"
    curl $des --data-urlencode $src --compressed

done

#do the build
echo "building.......\r"
curl 'http://192.168.49.1:8080/java/build/start' 
echo 'waiting for build.... \r'

curl 'http://192.168.49.1:8080/java/build/wait'

echo 'Done\r'
