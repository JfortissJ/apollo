# Running Bark-ML in Apollo

## Installation (everytime you start the docker container)
### Update Pip Package
`python3.7 -m pip install --upgrade pip --user`
### Install BARK ML
`python3.7 -m pip install bark-ml==0.4.24 --user`
To install a non-released version, use `python3.7 -m pip install git+https://github.com/bark-simulator/bark-ml --user` (and replace the url with the specific branch's url)
### Update Protobuf if you experience some weird Tensorflow Bug.
`sudo pip uninstall -y protobuf && sudo pip install protobuf` (uninstalls 3.1.0) and (installs 3.18.0 or newer)

## Starting BARK-ML Wrapper Node
`python3.7 modules/planning/planner/bark_rl_wrapper/bark_rl_wrapper.py`

## Planning with BARK-ML

### Simcontrol

Start Dreamview, activate Simcontrol, and launch the following modules:

- Routing: `cyber_launch start modules/routing/launch/routing.launch`
- Prediction: `cyber_launch start modules/tools/prediction/fake_prediction/fake_prediction.launch`
- Planning: `cyber_launch start modules/planning/launch/bark_rl_planning.launch`

Finally, you need to perform a routing. The planner should now start planning, and the car should drive.

Optionally, you can simulate obstacles via `cyber_launch start modules/fake_obstacle/launch/fake_obstacle.launch`. For this, you need to start the real Prediction node via `cyber_launch start modules/prediction/launch/prediction.launch`