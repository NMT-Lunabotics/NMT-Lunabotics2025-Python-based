#!/bin/bash

CONF="/etc/wifi-autoconnect.conf"

source "$CONF"

connect_wifi() {
    ssid="$1"
    psk="$2"
    nmcli dev wifi connect "$ssid" ${psk:+password "$psk"}
}

login_captive_portal() {
    ssid="$1"
    if [ "$ssid" = "$CAP_NMT_SSID" ]; then
        curl -s -L -d "${CAP_NMT_USERNAME_FIELD}=${CAP_NMT_USERNAME}&${CAP_NMT_PASSWORD_FIELD}=${CAP_NMT_PASSWORD}&${CAP_NMT_EXTRA_FIELDS}" "$CAP_NMT_LOGIN_URL" >/dev/null
    fi
}

check_internet() {
    curl -s --max-time 5 "$INTERNET_CHECK_URL" >/dev/null
}

while true; do
    connected_ssid=$(nmcli -t -f active,ssid dev wifi | grep "^yes:" | cut -d: -f2)
    if [ -n "$connected_ssid" ]; then
        if check_internet; then
            sleep "$SCAN_INTERVAL"
            continue
        else
            login_captive_portal "$connected_ssid"
            sleep "$SCAN_INTERVAL"
            continue
        fi
    fi

    found=$(nmcli -t -f ssid dev wifi | sort -u)
    if echo "$found" | grep -q "^$PRIORITY_SSID$"; then
        connect_wifi "$PRIORITY_SSID" "$PRIORITY_PSK"
        sleep "$CONNECT_TIMEOUT"
        continue
    fi

    for s in $FALLBACK_SSIDS; do
        if echo "$found" | grep -q "^$s$"; then
            connect_wifi "$s"
            sleep "$CONNECT_TIMEOUT"
            login_captive_portal "$s"
            break
        fi
    done

    sleep "$SCAN_INTERVAL"
done
