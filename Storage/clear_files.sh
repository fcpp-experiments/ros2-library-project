#!/bin/bash
find $(dirname "$0")/from_ap -type f -name '*.txt' -exec rm {} \;
find $(dirname "$0")/from_robot -type f -name '*.txt' -exec rm {} \;
find $(dirname "$0")/from_user -type f -name '*.txt' -exec rm {} \;
find $(dirname "$0")/from_wearable -type f -name '*.txt' -exec rm {} \;
