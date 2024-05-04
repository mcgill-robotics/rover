// line-graph.component.ts
import { Component, OnInit, ViewChild, ElementRef, OnDestroy } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';
import { Input } from '@angular/core';

@Component({
  selector: 'app-line-graph',
  templateUrl: './line-graph.component.html',
  styleUrls: ['./line-graph.component.scss']
})
export class LineGraphComponent implements OnInit, OnDestroy {

  @Input() topicName:string;

  @ViewChild('lineChart', { static: true }) canvasRef: ElementRef;
  private ctx: CanvasRenderingContext2D | null;
  private topic: ROSLIB.Topic;
  private ros: ROSLIB.Ros;
  private data: number[] = [3, 5, 2, 8, 4, 6];

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit(): void {
    this.topic = new ROSLIB.Topic({
      ros : this.ros,
      name : this.topicName, //to be changed to proper topic name
      messageType : 'std_msgs/Float32MultiArray'
    });

    this.ctx = (this.canvasRef.nativeElement as HTMLCanvasElement).getContext('2d');
    this.drawGraph();

    this.listen();
  }

  ngOnDestroy(): void {
    // Clean up resources (if any)
    // could store data to central
  }

  // methods:
  private listen() {
    this.topic.subscribe((message: any) => {
      this.data.push(message.data[0]);
      this.drawGraph();
      console.log(this.data);
    })
  }

  private drawGraph(): void {
    const width = this.ctx!.canvas.width;
    const height = this.ctx!.canvas.height;

    // Clear the canvas
    this.ctx!.clearRect(0, 0, width, height);

    // Calculate the gap between points
    const pointGap = width / (this.data.length - 1);

    // Set the line color and width
    this.ctx!.strokeStyle = '#FFF';
    this.ctx!.lineWidth = 2;

    // Draw horizontal grid lines and axis values
    const gridSpacing = 20;

    for (let i = gridSpacing; i < height; i += gridSpacing) {
      this.ctx!.beginPath();
      this.ctx!.moveTo(0, i);
      this.ctx!.lineTo(width, i);
      this.ctx!.stroke();
  
      // Add axis values vertical
      this.ctx!.fillStyle = '#FFF';
      this.ctx!.fillText((Math.floor((height - i)/gridSpacing)).toString(), 5, i - 5);
    }
  
    // Draw vertical grid lines and axis values
    for (let i = pointGap; i < width; i += pointGap) {
      this.ctx!.beginPath();
      this.ctx!.moveTo(i, 0);
      this.ctx!.lineTo(i, height);
      this.ctx!.stroke();
  
      // Add axis valuesv //not needed
      this.ctx!.fillText(Math.floor((i/pointGap )).toString(), i, height - 5);
    }


    // Begin the path
    this.ctx!.beginPath();

    this.ctx!.strokeStyle = '#2bc';
    // Move to the first point
    this.ctx!.moveTo(0, height - 20*this.data[0]); //need a factor to multiply

    // Draw lines connecting each point
    for (let i = 1; i < this.data.length; i++) {
      const x = i * pointGap;
      const y = height - 20*this.data[i];
      this.ctx!.lineTo(x, y);
    }

    // Stroke the path
    this.ctx!.stroke();
  }

}
