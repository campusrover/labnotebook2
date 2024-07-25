---
title: Tailscale VPN
date: July-22-2024
author: Pito Salas
status: dated
---
```

  100.89.2.122 cluster-1 \       / robot-1 100.89.2.122
                          \     /
  100.99.32.12 cluster-2 - - - - - robot-2 100.99.31.234
                           /    \
  100.88.77.234 cluster-3 /      \ robot-3 100.86.232.111

```

## Introduction

All our robots and cluster accounts have a common VPN enabled. In fact any other computers, like your laptop, can be added to the VPN. The effect of this is that they all will be on the same IP name space, that is, they can all communicate with each other via the IP. In fact the effect of enabling the VPN (Tailscale) is that you will see an exctra "virtual" network adapter when you type for example `ip addr`. That network adapter is created by tailscale.

### Tools

#### **tson**

To get a list of all the devices on our tailscale VPN you can use the `tson` alias.

```
$ tson
test3 - 100.73.71.98
ezimmerman - 100.83.124.19
hello.ts.net - 100.101.102.103
rpsalas - 100.73.134.92
otproject.taila146c.ts.net - 100.117.130.99
superset.taila146c.ts.net - 100.127.74.101
leejs8128 - 100.112.15.61
plat2.taila49c0.ts.net - 100.100.240.69
```
#### **tsoff**

To get a list of all the devices sometimes are on our tailscale VPN but are not right now you can use the `tsoff` alias.

#### **ip addr**

To see all the IP addresses associated with this computer, The output will include one entry for each network adapter. The ones that are currently connected to the network will have an ip address. 

```
$ ip addr
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP group default qlen 1000
    link/ether dc:a6:32:20:ac:14 brd ff:ff:ff:ff:ff:ff
    inet 192.168.5.51/22 brd 192.168.7.255 scope global dynamic eth0
       valid_lft 14124sec preferred_lft 14124sec
    inet6 2600:4040:52f4:5c00:dea6:32ff:fe20:ac14/64 scope global dynamic mngtmpaddr noprefixroute
       valid_lft 4561sec preferred_lft 4561sec
    inet6 fdb7:7ac:6f0f:1:dea6:32ff:fe20:ac14/64 scope global dynamic mngtmpaddr noprefixroute
       valid_lft 2591726sec preferred_lft 604526sec
    inet6 fe80::dea6:32ff:fe20:ac14/64 scope link
       valid_lft forever preferred_lft forever
3: wlan0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc fq_codel state DOWN group default qlen 1000
    link/ether dc:a6:32:20:ac:16 brd ff:ff:ff:ff:ff:ff
4: tailscale0: <POINTOPOINT,MULTICAST,NOARP,UP,LOWER_UP> mtu 1280 qdisc fq_codel state UNKNOWN group default qlen 500
    link/none
    inet 100.100.240.69/32 scope global tailscale0
       valid_lft forever preferred_lft forever
    inet6 fd7a:115c:a1e0::7264:f045/128 scope global
       valid_lft forever preferred_lft forever
    inet6 fe80::a3c7:d2ff:ae5:c63b/64 scope link stable-privacy
       valid_lft forever preferred_lft forever
```
### Setup

1. Make sure ~/rosutils is up to date by performing a git pull
2. Get a tailscale key from Pito if needed
3. `sudo ~/rosutils/pi_connect.sh [put ts key here]
4. On successful connection: `Connected. IP address: 100.xx.xxx.xxx`

