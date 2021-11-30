import pickle
from time import time
from bark.core.world.map import MapInterface

def main():
    pickle_file = "/apollo/data/log/experiment_2021-11-30-14-55-40/bark_serialization.2021-11-30-14-55-40.pickle"

    with open( pickle_file, "rb" ) as f:
        [scenario_history, driver_interaction_timesteps] = pickle.load(f)
    
    csvfile = '/apollo/modules/planning/data/base_map_lanes_guerickestr_assymetric_48.csv'
    print(csvfile)
    map_interface = MapInterface()
    map_interface.SetCsvMap(csvfile, 692000, 5.339e+06)

    for timestamp in scenario_history:
        print(timestamp)
        scenario = scenario_history[timestamp]
        scenario._map_interface = map_interface
        world = scenario.GetWorldState()
        for agent in world.agents.values():
            print(agent)
            

if __name__ == "__main__":
	main()
	print("Finished")
