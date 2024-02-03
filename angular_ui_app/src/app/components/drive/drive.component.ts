import { Component } from '@angular/core';
import { GamepadService } from '../../gamepad.service';

@Component({
  selector: 'app-drive-component',
  templateUrl: './drive.component.html',
  styleUrls: ['./drive.component.scss']
})
export class DriveComponent {
  constructor(private gamepadService: GamepadService) { }

  ngOnInit() {
    this.gamepadService.enable_gamepad();
  }
}