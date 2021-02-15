# remember chmod +x <filename>

# get propper directories and files
DIR=$(dirname "$0")
return_info="$(./$DIR/container_name.sh)"
return_info=( $return_info )
container_name="${return_info[0]}"
DIR="${return_info[1]}"
repo_DIR=$(dirname "$DIR")


echo "Directory of called script: $DIR"
echo "Name of container: $container_name"

docker build -t $container_name $repo_DIR/.
#docker build --build-arg HOME=${HOME} --build-arg repo_DIR="/${repo_DIR}" -t $container_name $repo_DIR/.
#docker run -it -d --net=host --name $container_name -v $HOME/repos/$container_name/src/:"/~/catkin_ws/src_extern/" $container_name

