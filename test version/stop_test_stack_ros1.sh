#!/bin/bash
# Stop every process of THIS test stack: supervisor, navigation pair, vision and the
# bundled driver check. Matching is by this folder's script paths only, so sensor
# drivers, roscore and other programs are never signalled.
# Optional $1: a PID whose ancestors are kept (the new launcher calling this script).
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
keep=" $$ "
pid="${1:-$$}"
while [ -n "$pid" ] && [ "$pid" -gt 1 ] 2>/dev/null; do
    keep="$keep$pid "
    pid=$(awk '{print $4}' "/proc/$pid/stat" 2>/dev/null) || break
done
SUPERVISOR="$TASK_DIR/start_all_ros1[.]sh"
ALL="$TASK_DIR/(start_all_ros1|start_navigation_ros1|start_server)[.]sh|$TASK_DIR/(navigation_ros1|collision_monitor_ros1|jetauto_find_object)[.]py"

matching() {
    ps -eo pid=,args= | while read -r candidate args; do
        case "$keep" in *" $candidate "*) continue ;; esac
        printf '%s\n' "$args" | grep -Eq -- "$1" && echo "$candidate"
    done
}

wait_gone() {
    local attempt
    for attempt in $(seq 1 "$2"); do
        [ -z "$(matching "$1")" ] && return 0
        sleep 1
    done
    return 1
}

# Supervisors first: their trap stops navigation before vision, and the Collision
# Monitor publishes zero velocity on SIGTERM.
pids=$(matching "$SUPERVISOR")
if [ -n "$pids" ]; then
    echo "Stopping previous test supervisor:" $pids
    kill -TERM $pids 2>/dev/null
    wait_gone "$SUPERVISOR" 8 || true
fi
# Orphans: e.g. a driver check whose parent was killed and still holds its lock.
pids=$(matching "$ALL")
if [ -n "$pids" ]; then
    echo "Stopping leftover test processes:" $pids
    kill -TERM $pids 2>/dev/null
    if ! wait_gone "$ALL" 6; then
        pids=$(matching "$ALL")
        echo "Force-stopping:" $pids
        [ -z "$pids" ] || kill -KILL $pids 2>/dev/null
        wait_gone "$ALL" 3 || true
    fi
fi
exit 0
