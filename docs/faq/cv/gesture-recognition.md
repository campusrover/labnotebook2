---
title: Gesture Recognition
author: Leo Gao
date: Dec 10 2024
---
# Using OpenCV and Neural Networks for hand landmarking with Google MediaPipe
### Leo Gao

Here is the process of utilizing raspicam's images to get recognized as gestures. 

# Summary
## After you have properly subscribed to the camera:
```python
rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_cb)
```
## This would be how you would convert a compressed image message into an OpenCV-readable format:

```python
def image_cb(self, msg):
        """Process incoming images from the camera."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.cv_image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")

```


## For each detected hand, calculate the pixel coordinates of the landmarks relative to the image dimensions, keep in mind that it is one hand at a time:

```python
def calc_landmark_list(self, image, hand_landmarks):
    image_width, image_height = image.shape[1], image.shape[0]
    return [
        [
            int(landmark.x * image_width),
            int(landmark.y * image_height),
        ]
        for landmark in hand_landmarks.landmark
    ]

```

## Normalize the coordinates relative to the wrist (first landmark) and scale them:

```python
def preprocess_landmarks(self, landmark_list):
    base_x, base_y = landmark_list[0]
    for landmark in landmark_list:
        landmark[0] -= base_x
        landmark[1] -= base_y

    flattened = list(itertools.chain.from_iterable(landmark_list))
    max_value = max(map(abs, flattened))
    normalized = [n / max_value for n in flattened]

    return normalized


```

## Draw landmarks for the detected hand :
```python 

def draw_landmarks(self, image, landmark_list):
    for point in landmark_list:
        cv.circle(image, tuple(point), 5, (255, 0, 0), -1)

```

## Combining it together by first tidying up the image by flipping it and then converting from BGR to RGB for mediapipe you would be able to achieve your processed gesture:

```python
def process_frame(self):
        image = cv.flip(self.cv_image, 1)  # Mirror image for convenience
        debug_image = copy.deepcopy(image)

        # Convert image to RGB for MediaPipe
        rgb_image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        results = self.hands.process(rgb_image)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Process hand landmarks
                landmark_list = self.calc_landmark_list(image, hand_landmarks)
                preprocessed_landmarks = self.preprocess_landmarks(landmark_list)

                # Classify gesture
                hand_sign_id = self.keypoint_classifier(preprocessed_landmarks)
                hand_sign_label = self.keypoint_classifier_labels[hand_sign_id]

                # Annotate image
                brect = self.calc_bounding_rect(image, hand_landmarks)
                debug_image = self.draw_landmarks(debug_image, landmark_list)
                debug_image = self.draw_bounding_rect(debug_image, brect)
                debug_image = self.draw_info_text(debug_image, brect, hand_sign_label)

        cv.imshow("Gesture Recognition", debug_image)
        cv.waitKey(1)
```
## These would be the outputs to the neural network which would have been pre-trained by you. 

```python
        hand_sign_id = self.keypoint_classifier(preprocessed_landmarks)
        hand_sign_label = self.keypoint_classifier_labels[hand_sign_id]
```
preprocessed_landmarks: 
A normalized list of hand landmark coordinates is passed to the KeyPointClassifier.
These landmarks are detected using MediaPipe and processed to remove variations caused by hand size, orientation, and position.

self.keypoint_classifier:

This is a neural network thatwas trained on a dataset of hand landmarks corresponding to various gestures. 
Input: Normalized landmarks (e.g., [x1, y1, x2, y2, ..., x21, y21]).
Output: Gesture ID (e.g., 0 for "Fist").

When called, the neural network takes the normalized coordinates as input and outputs a gesture class ID (an integer representing a specific gesture) based on the input landmarks.


## The run method continuously checks for new frames in self.cv_image and calls process_frame when available:

```python
def run(self):
    rospy.loginfo("Gesture recognizer is running...")
    while not rospy.is_shutdown():
        if self.cv_image is not None:
            self.process_frame()
        else:
            rospy.loginfo_once("Waiting for camera feed...")
        rospy.sleep(0.01)
```





