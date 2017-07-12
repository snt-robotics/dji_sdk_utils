function record_topics {
  TOPICS=$1
  BUFFER_SIZE=$2
  DIRECTORY=$3

  if [[ -z "${DIRECTORY}" ]]; then
    echo "You must specify a directory to which rosbag will be saved!"
    exit
  fi

  if [[ -z "${TOPICS}" ]]; then
    echo "You must specify a list of topics for rosbag record!"
    exit
  fi

  if [[ -z "${BUFFER_SIZE}" ]]; then
    echo "You must specify a buffer size for rosbag record process!"
    exit
  fi

  echo "Creating directory ..."
  mkdir -p "${DIRECTORY}"

  if [ $? -eq 0 ]; then
    echo "Launching rosbag..."
    echo "Recording to directory -> " ${DIRECTORY}
    echo "Buffer size:" ${BUFFER_SIZE}
    echo "Topics selected:"
    echo ${TOPICS}
    rosbag record -b "${BUFFER_SIZE}" -o "${DIRECTORY}/" "${TOPICS}"
  else
    echo "Directory count not be created, do you have permissions ?"
  fi
}
