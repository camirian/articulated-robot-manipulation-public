#!/bin/bash
set -e

# Project Root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR%/scripts}"
cd "$PROJECT_ROOT"

# Log Files
SIM_LOG="verification_sim.log"
ROS_LOG="verification_ros.log"

echo "=== STARTING VERIFICATION TEST ==="
echo "1. Starting Isaac Sim (Headless)..."
export ISAAC_HEADLESS=true
ISAAC_PYTHON="${ISAAC_SIM_PYTHON:-$HOME/isaac-sim-4.5.0/python.sh}"
"$ISAAC_PYTHON" scripts/sim_setup.py > "$SIM_LOG" 2>&1 &
SIM_PID=$!

echo "   Waiting for Bridge to Initialize..."
# Wait for "[SUCCESS] Bridge evaluated" in log
for i in {1..120}; do
    if grep -q "Bridge evaluated" "$SIM_LOG"; then
        echo "   [OK] Bridge is active!"
        break
    fi
    sleep 1
    if ! kill -0 $SIM_PID 2>/dev/null; then
        echo "   [FAIL] Simulator crashed! Check $SIM_LOG"
        exit 1
    fi
done

echo "2. Checking ROS 2 Topics..."
source ros2_ws/install/setup.bash
if ros2 topic list | grep -q "/joint_states"; then
    echo "   [OK] /joint_states detected"
else
    echo "   [FAIL] /joint_states missing!"
    kill $SIM_PID
    exit 1
fi

echo "3. Launching Pick-and-Place Demo..."
ros2 launch simple_manipulation demo_recording.launch.py > "$ROS_LOG" 2>&1 &
ROS_PID=$!

echo "   Waiting for Sequence Completion (approx 60s)..."
# Wait for "DONE." in ROS log
SUCCESS=false
for i in {1..90}; do
    if grep -q "DONE." "$ROS_LOG"; then
        echo "   [OK] SEQUENCE COMPLETED SUCCESSFULLY!"
        SUCCESS=true
        break
    fi
    if grep -q "Motion Timed Out" "$ROS_LOG"; then
        echo "   [FAIL] Motion Timed Out! Check $ROS_LOG"
        break
    fi
    sleep 1
done

echo "=== TEST RESULT ==="
if [ "$SUCCESS" = true ]; then
    echo "✅ VERIFICATION PASSED: The full pick-and-place sequence executed correctly."
else
    echo "❌ VERIFICATION FAILED: Did not complete in time."
    echo "--- TAIL OF ROS LOG ---"
    tail -n 20 "$ROS_LOG"
fi

# Cleanup
echo "Cleaning up processes..."
kill $ROS_PID 2>/dev/null || true
kill $SIM_PID 2>/dev/null || true
pkill -f "move_group" || true
pkill -f "rviz2" || true

exit 0
