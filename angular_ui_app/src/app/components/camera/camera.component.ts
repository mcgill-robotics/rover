import { Component, Input } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

@Component({
  selector: 'app-camera',
  templateUrl: './camera.component.html',
  styleUrls: ['./camera.component.scss']
})
export class CameraComponent {
  @Input() camera_size: string;
  @Input() camId: string;

  cameraSelection: ROSLIB.Topic;
  availableCamerasTopic: ROSLIB.Topic;
  rotationDeg: number = 0;
  inputCamera: string = '';
  availableCameras: string[] = ['No Camera']; // Default option when no cameras are available
  ros: ROSLIB.Ros;

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    // Subscribe to the topic that provides available camera names
    this.availableCamerasTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/available_cameras', // Topic that will provide the list of camera names
      messageType: 'std_msgs/String' // Assuming the camera names are sent as a comma-separated string
    });

    this.availableCamerasTopic.subscribe((message: any) => {
      if (message.data) {
        this.availableCameras = message.data.split(','); // Populate the array with camera names
      } else {
        this.availableCameras = ['No Camera'];
      }

      // Automatically select the first available camera or "No Camera" if none are available
      this.inputCamera = this.availableCameras[0];
      this.changeCamera();
    });

    // Initialize the camera selection topic
    this.cameraSelection = new ROSLIB.Topic({
      ros: this.ros,
      name: '/camera_selection_' + this.camId,
      messageType: 'std_msgs/String' // Changing to String since camera names are strings
    });
  }

  changeCamera() {
    this.cameraSelection.publish(new ROSLIB.Message({ data: this.inputCamera }));
  }

  flipCamera() {
    this.rotationDeg += 180;
    this.rotationDeg = this.rotationDeg % 360;
  }
}