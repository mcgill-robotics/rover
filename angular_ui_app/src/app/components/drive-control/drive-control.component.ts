import { Component, Input, Output, EventEmitter} from '@angular/core';

@Component({
  selector: 'app-drive-control-page',
  templateUrl: './drive-control.component.html',
  styleUrls: ['./drive-control.component.scss']
})
export class DriveControlComponent {
  @Input() initialValue: number = 0;
  @Output() counterChange = new EventEmitter<number>();

  counter: number = 0.00;

  ngOnInit() {
    this.counter = this.initialValue;
  }

  increment() {
    this.counter += 0.5;
    this.counterChange.emit(this.counter);
  }

  decrement() {
    this.counter -= 0.5;
    this.counterChange.emit(this.counter);
  }
}


