---
title: Ubuntu Performance
---
The following are a few techniques that have been known to speed up ubuntu on Raspberry Pi. These are not all verified or guaranteed.

## Figure out why  boot sequence is so slow
 figure out what each step during boot takes
```
systemd-analyze blame
```
## Check out snapd services
```
snap list
Snap services
```

## Remove multipath-service

This is safe on a raspberry Pi

```
sudo systemctl disable multipathd.service
```

## Disable cloud services
Which we dont need
```
sudo systemctl disable cloud-init.service
sudo systemctl disable cloud-init-local.service
sudo systemctl disable cloud-config.service
sudo systemctl disable cloud-final.service
```

Disable motd: /etc/default/motd-news
https://docs.vultr.com/working-with-the-ubuntu-message-of-the-day-motd-service

```
sudo systemctl disable apt-daily.service
sudo systemctl disable apt-daily.timer

sudo systemctl disable apt-daily-upgrade.timer
sudo systemctl disable apt-daily-upgrade.service
```
