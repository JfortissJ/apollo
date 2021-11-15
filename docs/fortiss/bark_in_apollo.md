# Running Bark-ML in Apollo

### Update Pip Package
python3.7 -m pip install --upgrade pip --user
### Install BARK ML
python3.7 -m pip install bark-ml --user
### Install Gym (currently apparently not a dependency of BARK ML)
python3.7 -m pip install gym --user
### Update Protobuf if you experience some weird Tensorflow Bug.
sudo pip uninstall protobuf
sudo pip install protobuf
