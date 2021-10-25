# Running Bark-ML in Apollo

## Installation (everytime you start the docker container)
### Update Pip Package
python3.7 -m pip install --upgrade pip --user
### Install BARK ML
python3.7 -m pip install bark-ml --user
### Install Gym (currently apparently not a dependency of BARK ML)
python3.7 -m pip install gym --user
### Update Protobuf if you experience some weird Tensorflow Bug.
sudo pip uninstall protobuf
sudo pip install protobuf


## Starting BARK-ML Wrapper Node
`python3.7 modules/planning/planner/bark_rl_wrapper/bark_rl_wrapper.py`

## Planning with BARK-ML

### Simcontrol

Start Dreamview, activate Simcontrol, and launch the following modules:

- Routing: `cyber_launch start modules/routing/launch/routing.launch`
- Prediction: `cyber_launch start modules/tools/prediction/fake_prediction/fake_prediction.launch`
- Planning: `cyber_launch start modules/planning/launch/bark_rl_planning.launch`

Finally, you need to perform a routing. The planner should now start planning, and the car should drive.

Optionally, you can simulate obstacles via `cyber_launch start modules/fake_obstacle/launch/fake_obstacle.launch`