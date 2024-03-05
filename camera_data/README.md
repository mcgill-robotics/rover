# Camera Data Module

This module read and writes data from available camera feeds to the `/camera_frames` topic.

### Messages

- `v_angle`: Vertical orientation tilt of the camera enclosed in [-90, 90] degrees (The pitch axis of rotation).
- `h_angle`: Horizontal orientation tilt of the camera enclosed in [-90, 90] degrees (The Yaw axis of rotation).

### Scripts

- `camera_feeds.py`: Collects available camera ports and outputs them all in distinct windows.
- `camera_subscriber.py`: Subscriber to the `/camera_frames` topic that displays the images to screen.
- `camera_publisher.py`: Publishes frames from the selected camera (from the `camera_selection` topic) to the
    `/camera_frames` topic at a rate of __30fps__.

### Tests

- `test_get_cameras.py`: Tests the camera feed collection from the `camera_feeds.py` script.
