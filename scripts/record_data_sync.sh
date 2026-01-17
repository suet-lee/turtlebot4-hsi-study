if ! [[ "$1" =~ ^[0-9]+$ ]]; then
    echo "Please provide a trial ID."
    exit 1
fi

ros2 bag record -o trial_$1 --topics /team_info/team0 \
    /team_info/team1 \
    /rigid_bodies \
    /team_info/team0/out \
    /team_info/team1/out \
    /team_info/sync_success

timestamp=$(date +%s)
mv trial_$1 ../data/trial_$1_$timestamp