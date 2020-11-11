# HD Map Generation

## Recordings with reasonable data
* fs01/KF/AS/# fortiss car/recordings/Recordings/2019-08-19/20190819152635.record.00002


## How to
1. Record a bag including a localization, eg. 20190710115340.record.00000
2. python modules/tools/map_gen/extract_path.py test.txt Recordings/2019-08-19/20190819152635.record.00002
   In case the record is split in several files: Pass all of them, eg. python modules/tools/map_gen/extract_path.py test.txt 20190911151106.record.00000 20190911151106.record.00001 20190911151106.record.00002 20190911151106.record.00003 20190911151106.record.00004
3. Alternativly: a or b (atm I only use 3b)
3a. python modules/tools/map_gen/map_gen.py test.txt and rename output to rename to base_map.txt
3b. python modules/tools/map_gen/map_gen_single_lane.py test.txt base_map.txt 0
4. new folder in modules/map/data/my_first_map, copy base_map.txt here via mv base_map.txt modules/map/data/parkplatz/base_map.txt
5. scripts/generate_routing_topo_graph.sh --map_dir modules/map/data/guerickestrasse/
6. bazel-bin/modules/map/tools/sim_map_generator --map_dir=modules/map/data/guerickestrasse/ --output_dir=modules/map/data/guerickestrasse/

7. View map: python modules/tools/mapshow/mapshow.py -m modules/map/data/guerickestrasse/base_map.txt
8.  Stop and Start Dreamview: new map my_first_map available

## Pitfalls
* Make sure the localization signal is in UTM and m: The hd maps that come with apollo are in UTM coordinates in m

## Apollo Documentation
* /apollo/modules/map/data/README.md

## Also
* See ticket: https://git.fortiss.org/fav/apollo35/issues/21

## How the map is visualied
* Yellow bold lines: road boundaries
* Dashed or solid white lines: lane boundaries
* Green thin solid line: reference line for planning