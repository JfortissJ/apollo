### Update Pip Package
python3.7 -m pip install --upgrade pip --user
### Install BARK ML
python3.7 -m pip install --upgrade bark-ml==0.4.32 --user

### Update Protobuf if you experience some weird Tensorflow Bug. (uninstalls 3.1.0 and installs 3.18.0 or newer)
sudo pip uninstall -y protobuf
sudo pip install protobuf