---
title: Diagnosing DNS Problems
author: Pito Salas
date: mar-2024
status: current
description: Hints and tips to get out of DNS hell
---
# DNS Problems

Sometimes, you can connect to your robot over ssh, so you know that the wifi is working correctly. And yet, git push and other commands are failing. This might be a problem DNS lookups.

## Checking

There are many ways to check. One that checks specifically DNS is:
```BashLexer
nslookup google.com
```
If this command hangs and times out with an error, this is good evidence that you have a problem with the DNS service.

Next check if the DNS service is working on your robot:
```
systemd-resolve --status
```
If that gives an error, then for some reason your DNS service has stopped. Try to restart it like this:

## Restarting the service
```
sudo systemctl restart systemd-resolved
```

## Verifying
If things are working you will see output like: 

```python
nslookup google.com # (1)
Server:         127.0.0.53
Address:        127.0.0.53#53

Non-authoritative answer:
Name:   google.com
Address: 142.250.80.78
Name:   google.com
Address: 2607:f8b0:4006:80c::200e

```
1: foobar
