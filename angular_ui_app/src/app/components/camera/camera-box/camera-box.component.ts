import { Component } from '@angular/core';
import { RosService } from 'src/app/ros.service';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-camera-box',
  templateUrl: './camera-box.component.html',
  styleUrls: ['./camera-box.component.scss']
})
export class CameraBoxComponent {
  isHovered: boolean = true;
  ros: ROSLIB.Ros;

  public imageSrc: string;

  constructor(
    private rosService: RosService
  ) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    new ROSLIB.Topic({
      ros: this.ros,
      name: '/usb_cam/image_raw/compressed',
      messageType: 'sensor_msgs/CompressedImage'
    }).subscribe((message: any) => {
      this.imageSrc = 'data:image/jpeg;base64,' + message.data;
    });
  }
}