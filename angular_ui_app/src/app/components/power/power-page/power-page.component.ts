import { NoopAnimationPlayer } from '@angular/animations';
import { Component } from '@angular/core';

@Component({
  selector: 'app-power-page',
  templateUrl: './power-page.component.html',
  styleUrls: ['./power-page.component.scss']
})
export class PowerPageComponent {
  b1:{
    name: string,
    value: number,
  };
  b2: number;
  s1: boolean;
  s2: boolean;
  s3: boolean;
  s4: boolean;
  sys: boolean[];
  pd: number[];

  constructor() {
    this.sys = [false,false,false, false];
    this.pd = [1,2,3,4,5,6,7,8];
    this.b1 = {name:"Battery 1", value:100};
  }
  
  toggleSys(i:number) {
    console.log(i);
    // console.log(this.sys[i]);
    
    this.sys[i] = !this.sys[i];
    // console.log(this.sys[i]);
  }


}
