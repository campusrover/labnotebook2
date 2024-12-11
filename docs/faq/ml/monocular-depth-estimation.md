---
title: Monocular Depth Estimation (No Depth Camera, One Camera Only)
author: Jeffrey Wang
date: Dec 10 2024
---

Let's suppose you don't have/don't want to use a depth camera, and only have one camera at your disposal. How could you possibly find the depth of an image? Shockingly, there exist machine learning models that can not only perform this task but can also do so with relative accuracy and speed. 

## Tutorial

We will use the MiDaS model for this tutorial for pytorch. 

In your code, do the following:
```     midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

        device = torch.device("cpu")
        midas.to(device)
        midas.eval()

        transform = midas_transforms.small_transform
```
This loads the model and necessary image transformers. This is recommended to be put into the top of a class, so that it only needs to be loaded once. 
It is not necessary to use MiDaS_small, you can choose whatever model you want. However, this one worked particularly well and without significant lag.

The device MUST be cpu, unless you are running this on a device with a Nvidia GPU.

Next, we apply transforms to our image before predicting:
```     input_batch = transform(cv_image).to(self.device)

        with torch.no_grad():
            prediction = midas(input_batch)

            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=cv_image.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()
```

Note that the image must be a CV image, so don't forget to do ```cv_bridge.imgmsg_to_cv2(image)``` on the image you are analyzing.

Save your prediction with ```output = prediction.cpu().numpy()```.

The model outputs a 2D array, where the element at the jth row, ith column represents the pixel at (i, j), if the image is starting from top left coordinates. When displaying the image, don't forget to normalize the pixels because it will display as grayscale unless you convert to RGB. The easiest way to do this is to divide the whole array by 1200, which is simply ```output_array / 1200``` because it is a numpy array.

## Details
This is a disparity map, not a depth map. This means the "depth" values at each pixel are NOT accurate and DO NOT accurately represent space in the real world, but rather represent relative depths to other objects. This means an object 10 inches away in real life may either be at "1000 units" or "400 units" depending on how close or far away other objects are. There are many claims of real-time monocular depth estimation with metric depth (meaning accurate, real life measurements) but they have all been tested by us to either be extremely laggy or not accurate. 