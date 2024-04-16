#!/bin/bash

# Only if needed
./make.sh clean

echo "Compiling target ap_engine..."

# Run AP with simulator
AP_ALG_USED=GREEDY AP_ROUND_PERIOD=0.5 AP_ROBOT_COUNT=10 AP_SIMULATOR_OFFSET_X=0.0 ./make.sh gui run -O -DAP_ENGINE_DEBUG=true ap_engine