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

  cameraSelection: ROSLIB.Topic;

  inputCamera: string = '';
  t_val = false;

  ros: ROSLIB.Ros;

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    this.cameraSelection = new ROSLIB.Topic({
      ros: this.ros,
      name: '/camera_selection',
      messageType: 'std_msgs/Int16'
    });
  }

  changeCamera() {
    this.cameraSelection.publish(new ROSLIB.Message({data: parseInt(this.inputCamera)}));
  }
}
