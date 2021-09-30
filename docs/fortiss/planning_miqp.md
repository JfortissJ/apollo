## Planning with MINIVAN (Mixed INteger InteractiVe plAnNing)

### Simcontrol

We here assume that you have successfully build Apollo and the respective planning_miqp module, see docs/fortiss/build.md.

You then start Dreamview, activate Simcontrol, and launch the following modules:

- Routing: `cyber_launch start modules/routing/launch/routing.launch`
- Prediction: `cyber_launch start modules/tools/prediction/fake_prediction/fake_prediction.launch`
- Planning: `cyber_launch start modules/planning/launch/miqp_planning.launch`

Finally, you need to perform a routing. The planner should now start planning, and the car should drive.

Optionally, you can simulate obstacles via `cyber_launch start modules/fake_obstacle/launch/fake_obstacle.launch`

Tested with https://github.com/bark-simulator/planner-miqp/commit/cea69161e7763af59d76bad87d8b4d6e4d0d1a8c
