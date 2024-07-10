---
title: "Camera Fails to initialize"
description: For advanced users who are trying to tix weird problems
author: Pito Salas
date: may-2023
status: new
type: faq
---
# Camera fails to initialize:

Ensure that the camera is enabled and there is enough GPU memory, Note that there isa config.txt in /boot/firmware when ubuntu is running but editing that seems to not do anything. Instead shut down the raspberry pi and pull the MicroSD card and plug it into your laptop so you can view it without ubuntu running.

1. Locate the file config.txt in the boot microsd when viewing it on your laptop.

1. Add the following lines to the `config.txt` file if they are not already there.

```
start_x=1
gpu_mem=128

```

And then reboot. Note that the config.txt file will say that you should not modify it and instead make the changes in a file called userconfig.txt. However I found that the userconfig.txt file is not invoked. So I added the lines above directly to config.txt.

1. Check the amount of gpu_memory

```
vcgencmd get_mem gpu
```
It should show 256 or whatever number you put there.

1. Check that the camera is supported and detected:

```
vcgencmd get_camera
```