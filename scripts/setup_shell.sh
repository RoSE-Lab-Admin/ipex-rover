#!/usr/bin/env bash
# One-time setup: make `roxey` and `ros2` available in every new terminal on
# the rover (and talk to the rover's ROS domain), without manual sourcing.
#
#   bash ~/ipex_ws/scripts/setup_shell.sh
#
# Adds a marked block to ~/.bashrc (safe to run again; it replaces the block).
set -e

WS="${1:-$HOME/ipex_ws}"
RC="$HOME/.bashrc"
BEGIN="# >>> roxey shell setup >>>"
END="# <<< roxey shell setup <<<"

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "No build found at $WS/install. Build first: cd $WS && colcon build --symlink-install"
  exit 1
fi

# Remove an existing block, then append a fresh one.
if grep -q "$BEGIN" "$RC" 2>/dev/null; then
  sed -i "/$BEGIN/,/$END/d" "$RC"
fi

cat >> "$RC" <<EOF
$BEGIN
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
export ROS_DOMAIN_ID=42
$END
EOF

echo "Added roxey setup to $RC. Open a new terminal (or run: source $RC), then try: roxey prims"
