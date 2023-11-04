import { Component } from '@angular/core';
// import { data } from 'jquery';
// import * as $ from "jquery";
// import * as jQuery from 'jquery';


import * as $ from 'jquery';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

// window['$'] = window['jQuery'] = $;



@Component({
  selector: 'app-button1',
  templateUrl: './button1.component.html',
  styleUrls: ['./button1.component.scss']
})
export class Button1Component {
  counter: number = 0;
// Ros service
  ros: ROSLIB.Ros;
  // addTwoIntsClient: ROSLIB.Service;
  service_client: ROSLIB.Service;
  request: ROSLIB.ServiceRequest;

  // sub
  a_k: ROSLIB.Topic;





  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit(): void {

    // this.addTwoIntsClient = new ROSLIB.Service({
    //   ros : this.ros,
    //   name : '/add_two_ints',
    //   serviceType : 'rospy_tutorials/AddTwoInts'
    // });
    this.service_client = new ROSLIB.Service({
      ros : this.ros,
      name : '/new_service_server',
      serviceType : 'beginner_tutorials/newS'
    });
  
    this.request = new ROSLIB.ServiceRequest({
      x : 4,
      y : 9
    });


    this.a_k = new ROSLIB.Topic({
      ros: this.ros,
      name: 'myNodePub',
      messageType: 'std_msgs/Float32MultiArray'
    })
    
    this.a_k.subscribe((message: any) => {
      console.log(message.data);
      // this.a_k.unsubscribe();
    });
    // this.addTwoIntsClient.callService(this.request, (result) => {
    //   console.log('Result for service call on '
    //     + this.addTwoIntsClient.name
    //     + ': '
    //     + result.sum);
    // });
  }

  increment() {
    // console.log("calling ajax")
    console.log("calling service")


    this.service_client.callService(this.request, (result) => {
      console.log('Result for service call on '
        + this.service_client.name
        + ': '
        + result.r);
    });
    // $.ajax({
    //   type: "POST",
    //   // url: "./count.py",
    //   // /home/blacklotus8/myAngular/src/angular_ui_app/src/app/sandbox/buttons/button1/count.py
    //   // url:"~/myAngular/src/angular_ui_app/src/app/sandbox/buttons/button1/count.py",
    //   url:"home/blacklotus8/myAngular/src/angular_ui_app/src/app/sandbox/buttons/button1/count.py",
      
    //   data: {param: this.counter}
    // }).done(function(m) {
    //   console.log("return of ajax")
    //   console.log(m);
    //   // console.log(data);
    // })
  }
  


}
