#!/bin/bash

if [ $# -eq 0 ]; then
    echo "Usage: $0 \"<goal_info>\" (ex: \"1.0;2.0;0.0\")"
    exit 1
fi

timestamp=$(date +%s%3N)

goal_id="GOAL-$timestamp"
goal_info="$1"
goal_file="from_user/goals/goal_$timestamp.txt"

echo "GOAL;$goal_id;0.0;0.0;0.0;$goal_info" > "$goal_file"

echo "File created $goal_file with id: $goal_id"