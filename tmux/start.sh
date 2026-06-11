#!/bin/bash
# Plain-tmux session script (one window per section below; panes are split and
# their commands typed via send-keys). Edit the send-keys lines to change what
# runs in each pane; ./kill.sh stops everything.

SESSION_NAME=simulation

# absolute path to this script's directory; every pane starts here
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

# commands executed first in every pane
PRE_WINDOW='export UAV_NAME=uav1; export RUN_TYPE=realworld; export UAV_TYPE=tello; export UAV_MASS=1.0'
SETUP="cd $SCRIPTPATH; $PRE_WINDOW"

if [ -n "$TMUX" ]; then
  echo "Already inside tmux, detach first."
  exit 1
fi

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  echo "Session $SESSION_NAME already exists; attach with 'tmux a -t $SESSION_NAME' or stop it with ./kill.sh."
  exit 1
fi

# ---------------- window: roscore ----------------
read W_roscore P <<< "$(tmux new-session -d -s "$SESSION_NAME" -n "roscore" -x 250 -y 50 -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'roscore' Enter
P=$(tmux split-window -t "$W_roscore" -P -F '#{pane_id}')
tmux select-layout -t "$W_roscore" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForRos; rosparam set use_sim_time false' Enter
tmux select-layout -t "$W_roscore" tiled

# ---------------- window: rosbag ----------------
read W_rosbag P <<< "$(tmux new-window -t "$SESSION_NAME" -n "rosbag" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForOffboard; ./record.sh' Enter
tmux select-layout -t "$W_rosbag" tiled

# ---------------- window: hw_api ----------------
read W_hw_api P <<< "$(tmux new-window -t "$SESSION_NAME" -n "hw_api" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForTime; roslaunch pairs_uav_dji_tello_api api.launch' Enter
tmux select-layout -t "$W_hw_api" tiled

# ---------------- window: status ----------------
read W_status P <<< "$(tmux new-window -t "$SESSION_NAME" -n "status" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; roslaunch pairs_uav_status status.launch' Enter
tmux select-layout -t "$W_status" tiled

# ---------------- window: control ----------------
read W_control P <<< "$(tmux new-window -t "$SESSION_NAME" -n "control" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; roslaunch pairs_uav_core core.launch platform_config:=`rospack find pairs_uav_dji_tello_api`/config/pairs_uav_system/tello.yaml custom_config:=./config/custom_config.yaml world_config:=./config/world.yaml network_config:=./config/network_config.yaml' Enter
tmux select-layout -t "$W_control" tiled

# ---------------- window: takeoff ----------------
read W_takeoff P <<< "$(tmux new-window -t "$SESSION_NAME" -n "takeoff" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'history -s rosservice call /uav1/hw_api/arming 1' Enter
P=$(tmux split-window -t "$W_takeoff" -P -F '#{pane_id}')
tmux select-layout -t "$W_takeoff" even-vertical
tmux send-keys -t "$P" "$SETUP; "'history -s rosservice call /uav1/tellopy_wrapper/zero' Enter
P=$(tmux split-window -t "$W_takeoff" -P -F '#{pane_id}')
tmux select-layout -t "$W_takeoff" even-vertical
tmux send-keys -t "$P" "$SETUP; "'history -s rosservice call /uav1/uav_manager/midair_activation' Enter
tmux select-layout -t "$W_takeoff" even-vertical

# ---------------- window: throw_and_go ----------------
read W_throw_and_go P <<< "$(tmux new-window -t "$SESSION_NAME" -n "throw_and_go" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'history -s rosservice call /uav1/tellopy_wrapper/throw' Enter
P=$(tmux split-window -t "$W_throw_and_go" -P -F '#{pane_id}')
tmux select-layout -t "$W_throw_and_go" even-vertical
tmux send-keys -t "$P" "$SETUP; "'history -s rosservice call /uav1/tellopy_wrapper/zero' Enter
P=$(tmux split-window -t "$W_throw_and_go" -P -F '#{pane_id}')
tmux select-layout -t "$W_throw_and_go" even-vertical
tmux send-keys -t "$P" "$SETUP; "'history -s rosservice call /uav1/uav_manager/midair_activation' Enter
tmux select-layout -t "$W_throw_and_go" even-vertical

# ---------------- window: land ----------------
read W_land P <<< "$(tmux new-window -t "$SESSION_NAME" -n "land" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'history -s rosservice call /uav1/hw_api/arming 0' Enter
tmux select-layout -t "$W_land" tiled

# ---------------- window: goto ----------------
read W_goto P <<< "$(tmux new-window -t "$SESSION_NAME" -n "goto" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'history -s rosservice call /$UAV_NAME/control_manager/goto_relative \"goal: \[0.0, -0.5, 0.0, 0.0\]\"' Enter
P=$(tmux split-window -t "$W_goto" -P -F '#{pane_id}')
tmux select-layout -t "$W_goto" tiled
tmux send-keys -t "$P" "$SETUP; "'history -s rosservice call /$UAV_NAME/control_manager/goto_relative \"goal: \[0.0, 0.5, 0.0, 0.0\]\"' Enter
tmux select-layout -t "$W_goto" tiled

# ---------------- window: rviz ----------------
read W_rviz P <<< "$(tmux new-window -t "$SESSION_NAME" -n "rviz" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForControl; rosrun rviz rviz -d ./rviz.rviz' Enter
P=$(tmux split-window -t "$W_rviz" -P -F '#{pane_id}')
tmux select-layout -t "$W_rviz" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch pairs_rviz_plugins load_robot.launch' Enter
P=$(tmux split-window -t "$W_rviz" -P -F '#{pane_id}')
tmux select-layout -t "$W_rviz" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForControl; waitForControl; roslaunch pairs_rviz_plugins rviz_interface.launch' Enter
tmux select-layout -t "$W_rviz" tiled

# ---------------- window: layout ----------------
read W_layout P <<< "$(tmux new-window -t "$SESSION_NAME" -n "layout" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForControl; sleep 3; ~/.i3/layout_manager.sh ./layout.json' Enter
tmux select-layout -t "$W_layout" tiled

# ---------------- window: kill (press enter inside to stop the session) ----------------
read W_kill P <<< "$(tmux new-window -t "$SESSION_NAME" -n "kill" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SCRIPTPATH/kill.sh"

# mouse support (select panes / scroll with the mouse)
tmux set-option -t "$SESSION_NAME" mouse on

tmux select-window -t "$W_control"
tmux -2 attach-session -t "$SESSION_NAME"
