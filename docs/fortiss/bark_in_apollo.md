# Running Bark-ML in Apollo

## Installation (everytime you start the docker container)
`bash scripts/fortiss/install_bark_ml.sh`

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