# How to drive on a map using simctrl

## Default Sunnyvale with two offices map

1. Load map in the upper right corner of dreamview
2. Launch Simctl
3. Launch Control, Planning, Routing in Module Controller Tab
4. Send a routing request in the Route Editing Tab
=> Vehicle is moving in the simulation


## Using a Custom Map
1. Load the map and reload it
2. Make sure to specify a valid start pose: 
   1. There are parameters for that in sim control: fortiss_simcontrol_set_start_pose, fortiss_simcontrol_start_x, fortiss_simcontrol_start_y
   2. Set these in the flagfile:  /apollo/modules/common/data/global_flagfile.txt
3. Launch Simcontrol (vehicle will jump to the specified start pose)
4. Launch Control, Planning, Routing in Module Controller Tab
5. Send a routing request in the Route Editing Tab or select default route in the lower left corner of the Tasks tab
=> Vehicle is moving in the simulation


##  Good to know

### If the Routing Tab is not available: 
- switch to sunnyvale map
- reload dreamview
- tab is here
- switch back to guerickestraße map

## Errors

### Errors that probably do nor yield a problem
- E0830 15:00:42.363376 22508 map_service.cc:371]  Unable to get paths from routing! -> is this a problem?
- E0830 14:57:17.676298 22507 map_service.cc:495]  Failed to find lane: 4 -> is this a problem?

### Problematic errors
- Only one or two lane segments are visualized in dreamview -> only a visualization issue! I can drive the last few meters on the map, but the map is not visualized!
- I cannot initially load a custom map, have to switch to a sunnyvale map and back. why?