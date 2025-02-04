#!/bin/bash

CLEAN=${1:-false}

if [ "$CLEAN" = "true" ]; then
    echo "Cleaning..."
    ./make.sh clean
fi

echo "Compiling target ap_engine..."

# Run AP with simulator
AP_ALG_USED=LAZY AP_ROUND_PERIOD=0.25 AP_ROBOT_COUNT=5 AP_SIM_KIOSK_X=3.5 AP_SIM_KIOSK_Y=0.0 ./make.sh gui run -O -DAP_ENGINE_DEBUG=true -DAP_COMM_RANGE=3.0 ap_engine