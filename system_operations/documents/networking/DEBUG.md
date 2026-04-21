# Networking Tools
### Network router console
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

### Status tools
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

### Restart networking services

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

***
<br><br><br><br><br>


# Networking Setup Guides

### Connect to network through terminal 

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

### Set network connection priority

**1. Set network priority, (higher number = higher priority)**
```
nmcli connection modify "NETWORK_NAME" connection.autoconnect-priority 10
```

**2. Ensure autoconnect is enabled**
```
nmcli connection modify "NETWORK_NAME" connection.autoconnect yes
```

**3. Restart networking**
```
nmcli connection down "NETWORK_NAME"
nmcli connection up "NETWORK_NAME"
```
***
<br>


### Set static ip address

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