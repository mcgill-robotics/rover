// line-graph.component.ts
import { Component, OnInit, ViewChild, ElementRef, OnChanges, SimpleChanges } from '@angular/core';
import { Input } from '@angular/core';

@Component({
  selector: 'app-graph',
  templateUrl: './graph.component.html',
  styleUrls: ['./graph.component.scss']
})
/* 
To be done: 
Optimization, value clearing 
*/ 

export class GraphComponent implements OnInit,  OnChanges {
  @Input() title:string;  
  @Input() y_axis:string;
  @Input() x_axis:string;
  @Input() type:string; 
  @Input() data:number[];

  @ViewChild('chart', { static: true }) canvasRef: ElementRef;
  private ctx: CanvasRenderingContext2D | null;

  width:number;

  height:number;


  ngOnInit(): void {
    this.ctx = (this.canvasRef.nativeElement as HTMLCanvasElement).getContext('2d');
    this.width = this.ctx!.canvas.width;
    this.height = this.ctx!.canvas.height;
  }

  // when data changes
  ngOnChanges(changes: SimpleChanges): void { //only tracks the object change rip, never ref
    switch (this.type) {
      case "histogram":
        this.drawHistogram();
        break;    
      case "line":
        this.drawLineChart();
        break;
      default:
        console.log("Wrong graph type");
    }
  }


  // provide a selector to choose the type of graph --later
  private drawHistogram(): void {
    // Clear the canvas
    this.ctx!.clearRect(0, 0, this.width, this.height);

    const drawHeight = this.height - 10; //create the offset
  
    // Find the minimum and maximum values in the dataset
    const minValue = Math.min(...this.data);
    const maxValue = Math.max(...this.data);
  
    // Calculate the range of values
    const valueRange = maxValue - minValue + 1;
    let factor = 1;
    if( valueRange > 100) {
      factor = Math.floor(valueRange/10);
    }
  
    // Initialize an array to store the frequency of each value
    const frequencies: number[] = new Array(valueRange).fill(0);
  
    // Count the occurrences of each value in the dataset
    this.data.forEach(value => {
      frequencies[value - minValue]++;
    });
  
    // Calculate the maximum frequency
    const maxFrequency = Math.max(...frequencies);
    
    // Calculate the this.width of each bar
    const barWidth = this.width / valueRange;
    
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
    if ((i + minValue) % factor == 0) {
      const label = (minValue + i).toString(); // Calculate the value for the label
      this.ctx!.fillText(label, x + barWidth / 2, drawHeight); // Draw label underneath the bar
    }
      
    }
  }
  
  

// top peak and line value to be fixed
  private drawLineChart(): void {
    // if only want last 20 dp
    // if (this.data.length > 20) {
    //   this.data = this.data.slice(-20);
    // } 

    let maxValue = Math.max(...this.data);
    let minValue = Math.min(...this.data);
    maxValue = Math.ceil(maxValue / 10) * 10;
    if (maxValue < 5) {maxValue = 10}; //prevent 0 value division in graph
    (minValue < 0) ? minValue = Math.floor(minValue / 10) * 10 : minValue = 0 ;
    let range = maxValue - minValue;

    // Clear the canvas
    this.ctx!.clearRect(0, 0, this.width, this.height);

    // Calculate the x gap between points
    const pointGap = this.width / (this.data.length - 1);

    // Set the line color and this.width
    this.ctx!.strokeStyle = '#FFF';
    this.ctx!.lineWidth = 1;

    // Draw horizontal grid lines and axis values
    let verticalSpace = Math.floor(this.height/10); //always same number of lines

    // initial offset
    for (let i = 20; i < this.height; i += verticalSpace) {
      this.ctx!.beginPath();
      this.ctx!.moveTo(0, i);
      this.ctx!.lineTo(this.width, i);
      this.ctx!.stroke();
  
      // Add vertical values 
      this.ctx!.fillStyle = '#FFF';
      this.ctx!.fillText((Math.ceil(((this.height - i)/this.height)*range + minValue)).toString(), 5, i - 5);
    }

    // draw X axis for value 0
    let xxx = this.height*maxValue/range;
    this.ctx!.beginPath();
    this.ctx!.strokeStyle = '#F00';
      this.ctx!.moveTo(0, xxx);
      this.ctx!.lineTo(this.width, xxx);
      this.ctx!.stroke();
  
    // Draw vertical grid lines and axis values
    for (let i = pointGap; i < this.width; i += pointGap) {
      this.ctx!.beginPath();
    this.ctx!.strokeStyle = '#FFF';
      this.ctx!.moveTo(i, 0);
      this.ctx!.lineTo(i, this.height);
      this.ctx!.stroke();
  
      // Add axis valuesv //not needed
      this.ctx!.fillText(Math.floor((i/pointGap )).toString(), i, this.height - 5);
    }


    // Begin the path
    this.ctx!.beginPath();

    this.ctx!.strokeStyle = '#2bc';
    // Move to the first point
    this.ctx!.moveTo(0, this.height - this.height*(this.data[0] - minValue)/range);

    // Draw lines connecting each point
    for (let i = 1; i < this.data.length; i++) {
      const x = i * pointGap;
      const y = (this.height - this.height*(this.data[i] - minValue)/range); //fix
      this.ctx!.lineTo(x, y);
    }

    // Stroke the path
    this.ctx!.stroke();
  }

}
