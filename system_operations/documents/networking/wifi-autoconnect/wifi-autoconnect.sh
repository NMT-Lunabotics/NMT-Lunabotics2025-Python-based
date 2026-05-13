#!/bin/bash

# Load config file
CONF="/etc/wifi-autoconnect.conf"
if [[ ! -f "$CONF" ]]; then
    echo "[ERROR] Missing config: $CONF"
    exit 1
fi
source "$CONF"

# System varibles
failure_count=0
last_recovery=0

# Connect to the network
connect_wifi() {
    echo "[INFO] Connecting to $PRIORITY_SSID"
    nmcli dev wifi connect "$PRIORITY_SSID" password "$PRIORITY_PSK"
}

# Check connection to the primary network
is_connected() {
    nmcli -t -f DEVICE,STATE dev status | grep -q "^${WIFI_INTERFACE}:connected"
}

# Check connection non-primary network
current_ssid() {
    nmcli -t -f ACTIVE,SSID dev wifi | grep '^yes' | cut -d: -f2
}

ssid_exists() {
    nmcli -t -f SSID dev wifi list | grep -Fxq "$PRIORITY_SSID"
}

# Restart network interface
interface_restart() {
    echo "[WARN] Restarting interface $WIFI_INTERFACE"
    sudo ip link set "$WIFI_INTERFACE" down
    sleep 2
    sudo ip link set "$WIFI_INTERFACE" up
}

# Toggle wifi on and then off
wifi_toggle() {
    echo "[WARN] Toggling Wi-Fi"
    nmcli radio wifi off
    sleep 2
    nmcli radio wifi on
}

run_recovery() {
    # Limit rate of recovery attempts 
    now=$(date +%s)
    if (( now - last_recovery < RECOVERY_COOLDOWN )); then
        echo "[WARN] Recovery cooldown active"
        return
    fi
    last_recovery=$now

    # Attempt recovery pattern from config order
    IFS=',' read -ra actions <<< "$RECOVERY_ACTIONS"
    for action in "${actions[@]}"; do
        case "$action" in
            reconnect) connect_wifi ;; interface_restart) interface_restart ;; wifi_toggle) wifi_toggle ;;
        esac

        # Between steps if router becomes active stop recovery pattern 
        sleep 2
        if ping -c 1 -W "$PING_TIMEOUT" "$ROUTER_IP" >/dev/null 2>&1; then
            echo "[INFO] Connection re-established"
            failure_count=0
            return 0 
        fi
    done
}


# Main control loop
while true; do
    ssid=$(current_ssid)

    # Constantly check connection status 
    if [[ "$ssid" == "$PRIORITY_SSID" ]]; then
        if ! ping -c 1 -W "$PING_TIMEOUT" "$ROUTER_IP" >/dev/null 2>&1; then
            echo "[ERROR] Router not responding"
            ((failure_count++))
        else
            echo "[INFO] Connection ok"
            failure_count=0
        fi
    else
        echo "[ERROR] Not connected to $PRIORITY_SSID"
        ((failure_count++))
        if ssid_exists; then connect_wifi; fi
    fi

    # Check if we are on a non primary backup connection
    if [[ "$ssid" != "$PRIORITY_SSID" ]]; then
        sleep "$CONNECTION_INTERVAL"
        continue
    fi

    # If no backup or primary network is found attempt recovery cycle
    if (( failure_count >= FAILURE_THRESHOLD )); then
        echo "[WARN] Attempting recovery"
        run_recovery
    fi
    sleep "$CONNECTION_INTERVAL"
done