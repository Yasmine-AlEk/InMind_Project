set -e

echo "---- Libraries used for installing software [~260MB]:"
apt update && apt install -y \
  cmake build-essential libcurl4-openssl-dev lsb-release bash-completion \
  wget unzip curl git \
  ssh rsync libglib2.0-dev \
  htop nano vim \
  python3-dev python3-pip \
  curl unzip openssh-client \
  python3-pip libsdbus-c++-dev
  
python3 -m pip install python-can flask websockets

ARCHITECTURE=$(uname -m)
echo "Found architecture ${ARCHITECTURE}"
