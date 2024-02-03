import { Component, Input, Output, EventEmitter} from '@angular/core';

@Component({
  selector: 'app-drive-control-page',
  templateUrl: './drive-control.component.html',
  styleUrls: ['./drive-control.component.scss']
})
export class DriveControlComponent {
  @Input() initialValue: number = 0;

  // array contains initial values 
  array  : number[]= [0,0,0];

  increment(id: number) {
    console.log("id: ", id);
    this.array[id] += 0.5;
    console.log("val 0: ", this.array[id]);
  }

  decrement(id: number) {
    this.array[id] -= 0.5;
  }
}


