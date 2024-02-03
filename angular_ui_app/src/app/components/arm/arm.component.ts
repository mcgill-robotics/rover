import { Component } from '@angular/core';

@Component({
  selector: 'app-arm-component',
  templateUrl: './arm.component.html',
  styleUrls: ['./arm.component.scss']
})
export class ArmComponent {
  joints: number[];
  test: string;
  error: boolean;

  constructor() {
    this.joints = [1,2,3,4,5,6,7,8,9];
    this.test = "hlfdkgjhsd";
    this.error = true;

  }
}
