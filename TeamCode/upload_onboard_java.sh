#keep this file inside of the BIG Teamcode folder
#cd to kuriosityrobotics2018/TeamCode
#to run in terminal enter: $ ./upload_onboard_java.sh

src_folder="src/main/java/org/firstinspires/ftc/teamcode"
des_folder="src/org/firstinspires/ftc/teamcode"
host_name="http://192.168.49.1:8080"

echo -e "Connecting to ${host_name}, make sure this is the right ip for the onbotjava."
echo -e "Deleting everything on onbotjava for ${des_folder}..."
curl '${host_name}/java/file/delete' --data 'delete=["${des_folder}"]' --compressed

for file in ${src_folder}/*
do
    echo -e $(basename $file)
    file_only=$(basename $file)    
    des="${host_name}/java/file/save?f=/${des_folder}/${file_only}"
    src="data@${src_folder}/${file_only}"
    echo -e $des
    echo -e $src
    echo -e "Uploading..."
    curl $des --data-urlencode $src --compressed

done

#do the build
echo -e " "
host_name_build="${host_name}/java/build/start"
echo -e "Building.......${host_name_build}"

curl $host_name_build

echo -e " "
echo -e 'Waiting for build....'

#curl '${host_name}/java/build/wait'

host_name_wait="${host_name}/java/build/wait"

curl $host_name_wait
echo -e 'Done'
