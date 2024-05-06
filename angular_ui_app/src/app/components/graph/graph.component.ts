// line-graph.component.ts
import { Component, OnInit, ViewChild, ElementRef, OnDestroy } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';
import { Input } from '@angular/core';

@Component({
  selector: 'app-graph',
  templateUrl: './graph.component.html',
  styleUrls: ['./graph.component.scss']
})
// this component may be converted into a generic graph  component
export class GraphComponent implements OnInit, OnDestroy {
  //params for the graph
  @Input() topicName:string;
  @Input() y_axis:string;
  @Input() x_axis:string;
  @Input() title:string;  
  @Input() type:string;  



  @ViewChild('chart', { static: true }) canvasRef: ElementRef;
  private ctx: CanvasRenderingContext2D | null;
  private topic: ROSLIB.Topic;
  private ros: ROSLIB.Ros;
  private data: number[] = [3, 2, 2, 8, 5, 6, 5, 2];
  // for histogram we can create a frequency array or make data[][]


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
    // this.drawGraph();  
    switch (this.type) {
      case "histogram":
        this.drawHistogram();
        break;    
      case "line":
        this.drawLineChart();
        break;
        
      defautt:
        console.log("Wrong graph type");
    }
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

      // this.drawLineChart();
      switch (this.type) {
        case "histrogram":
          this.drawHistogram();
          break;    
        case "line":
          this.drawLineChart();
          break;

        defautt:
          console.log("Wrong graph type");
      }

      // this.drawHistogram();
    })
  }

  // provide a selector to choose the type of graph --later
  private drawHistogram(): void {
    const width = this.ctx!.canvas.width;
    const height = this.ctx!.canvas.height;
       
    // Clear the canvas
    this.ctx!.clearRect(0, 0, width, height);

    const drawHeight = height - 10; //create the offset
  
    // Find the minimum and maximum values in the dataset
    const minValue = Math.min(...this.data);
    const maxValue = Math.max(...this.data);
  
    // Calculate the range of values
    const valueRange = maxValue - minValue + 1;
  
    // Initialize an array to store the frequency of each value
    const frequencies: number[] = new Array(valueRange).fill(0);
  
    // Count the occurrences of each value in the dataset
    this.data.forEach(value => {
      frequencies[value - minValue]++;
    });
  
    // Calculate the maximum frequency
    const maxFrequency = Math.max(...frequencies);
  
    // Set the color for the bars
    
    // Calculate the width of each bar
    const barWidth = width / valueRange;
    
    // Draw each bar and add labels
    for (let i = 0; i < valueRange; i++) {
      this.ctx!.fillStyle = '#3498db';
      const barHeight = (frequencies[i] / maxFrequency) * drawHeight; // Scale the bar drawHeight based on the maximum frequency
      const x = i * barWidth;
      const y = drawHeight - barHeight + 20;
      this.ctx!.fillRect(x, y, barWidth, barHeight);
  
     // Add count label at the top of the bar
     this.ctx!.fillStyle = '#fff'; // Set text color
     if (frequencies[i] != 0) {
       const countLabel = frequencies[i].toString(); // Get the count for the current bar
       this.ctx!.textAlign = 'center'; // Center-align the text
       this.ctx!.font = '12px Arial'; // Set font size and family
       this.ctx!.fillText(countLabel, x + barWidth / 2, y - 5); // Draw count label at the top of the bar
     }

    // Add label underneath the bar
    const label = (minValue + i).toString(); // Calculate the value for the label
    this.ctx!.fillText(label, x + barWidth / 2, drawHeight); // Draw label underneath the bar
      
    }
  }
  
  


  private drawLineChart(): void {
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
