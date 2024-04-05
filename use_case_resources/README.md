# How to run


## AP
### Library
To build:
```bash
cd /home/robot/Documenti/projects/NODES/PoC/AP_Engine

./make.sh gui build -O -DAP_COMM_RANGE=5 ap_engine
```

To run:
```bash
cd /home/robot/Documenti/projects/NODES/PoC/AP_Engine      
cd bin
AP_ALG_USED=GREEDY AP_ROUND_PERIOD=0.2 AP_WEARABLE_COUNT=0 AP_ROBOT_COUNT=5 AP_SIMULATOR_OFFSET_X=0.4 ./run/ap_engine
```

## Simulation
### Library
```bash
cd /home/robot/Documenti/projects/NODES/PoC
./turtlebot3_run.sh
```


### Out of order robot
```bash
# $ROBOT_NAME can be tb3_1, tb3_2 etc...
./out_of_order.sh $ROBOT_NAME
```