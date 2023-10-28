import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

@Component({
  selector: 'app-test-page',
  templateUrl: './test-page.component.html',
  styleUrls: ['./test-page.component.scss']
})
export class TestPageComponent {
  ros: ROSLIB.Ros;
  driveData: any = "placeHolderValue";

  drive_location_sub: ROSLIB.Topic;

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }


  ngOnInit(): void {
    this.drive_location_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/position_pose',
      messageType: 'geometry_msgs/Pose'
    });

    this.drive_location_sub.subscribe((message: any) => {
      // console.log(":runnung");
      console.log(message.orientation);
      console.log(message.position);

      this.driveData = message.position.x; //need to switch
    });

  }


}
