# Networking Tools
## Network router console
**Network router settings can be access at**
```
http://192.168.0.1
```
**or**
```
http://tplinkwifi.net/
```
***
<br>

## Status tools
**List status of network interfaces:**
```
nmcli dev status
```

**List nearby and networks:**
```
nmcli device wifi list
```

**List all saved networks:**
```
nmcli connection show
```

**List connectivity and routing infomation:**
```
ip route | grep default
ip route
```

**Show which interface is used for internet traffic**
```
ip route get 8.8.8.8
```

**List network interfaces and ip addresses:**
```
ip a
hostname -I
```

**Check if wifi is working by ping google dns:**
```
ping -c 3 8.8.8.8
```

**Check wifi strength:**
```
nmcli -f IN-USE,SSID,SIGNAL dev wifi list
```
***
<br>

## Connection monitoring
**Simple methiod to list UDP commuications:**
```
nc -ul 10000
```

**Advenced methiod to list UDP commuications:**
```
sudo tcpdump -i wlp4s0 udp port 10000
```
All heatbeat and telemetry commuications occor on port ``10000`` or ``11010``, consol commands occor on ``10001`` and normal commands occor on ``11000``
***
<br>

## Restart networking services

**Disconnect and reconnect to network:**
```
nmcli dev disconnect wlan0
nmcli dev wifi connect "SSID" password "PASSWORD"
```

**Restart networking:**
```
nmcli networking off
sleep 2
nmcli networking on
```

```
sudo systemctl restart NetworkManager
```
<br>

***
<br><br><br><br><br>















# Networking Setup Guides

## Connect to network through terminal 

**Add new network:**
```
nmcli device wifi connect "NETWORK_NAME" password "PASSWORD"
```

**Add hidden network:**
```
nmcli device wifi connect "SSID" password "PASSWORD" hidden yes
```
***
<br>

## Set network connection priority

**1. Set network priority, (higher number = higher priority)**
```
sudo nmcli connection modify "NETWORK_NAME" connection.autoconnect-priority 10
```

**2. Ensure autoconnect is enabled**
```
sudo nmcli connection modify "NETWORK_NAME" connection.autoconnect yes
```

**3. Restart networking**
```
nmcli connection down "NETWORK_NAME"
nmcli connection up "NETWORK_NAME"
```
***
<br>


## Set static ip address

**1. Find network interface to apply static ip to, i.e. (wlan0, eno1, or wlP1ps0), and it's name**
```
nmcli dev status
nmcli connection show
```

**2. Set the network static ip address, default gateway, and ensure network is in manual mode, (static ip=20, keep other commands the same)**
```
nmcli connection modify "CONNECTION_NAME" ipv4.addresses 192.168.0.20/24
nmcli connection modify "CONNECTION_NAME" ipv4.gateway 192.168.0.1
nmcli connection modify "CONNECTION_NAME" ipv4.method manual
```

**3. Set DNS, (Optional, can cause ROS issues)**
```
nmcli connection modify "NETWORK_NAME" ipv4.dns "8.8.8.8 1.1.1.1"
```

**4. Restart network**
```
nmcli connection down "NETWORK_NAME"
nmcli connection up "NETWORK_NAME"
```
***
<br>

## Change power saving settings

### Change mode temporality
**1a. Read current power save state**
```
iw dev wlP1p1s0 get power_save
```

**1b. Set current power save state temporality (on/off)**
```
sudo iw dev wlP1p1s0 set power_save off
```

**2. Restart network manager**
```
sudo systemctl restart NetworkManager
```
<br>

### Change mode permanently

**1. Set current power save state permanently (0=default, 1=enabled, 2=disabled, 3=ignore)**
```
sudo nano /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf
set to 2
```

**2. Restart network manager**
```
sudo systemctl restart NetworkManager
```
<br><br><br><br><br>















# Compeation rule guide

## General router setup
**1. Set router SSID to the assgined team number**
``
Team_##
``

**2. SSID must be broadcasted, SSID Broadcast must be on, and hidden network must be turned off**

**3. Encryption is required (WPA2 or WPA3), WPA2-PSK is a good choice**

**4. Disable channel bonding and set 2.4GHz band width to**
``
20MHz
``
<br>

**5. 5GHz band is allowed to be used during compeation and is more stable, but network band is not monitored. Usage of 2.4GHz recommended. Turn off all unsed bands**

## Robot pit router setup
**1. 2.4GHz should be turned off at all times in robot pit, either through an turned off router or by turning off the 2.4GHz band, 5GHz band is allowed**

**2. Pitboss may authorize 2.4GHz band checks on channel** ``11`` **For short time periods**

**3. During comm check quickly switch to channel** ``1`` **Then turn off router until we reach the comm test station**

## Compeation 
**1. During compeation runs channel** ``1`` **will be used for all commuication**