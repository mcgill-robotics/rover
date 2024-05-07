import { NoopAnimationPlayer } from '@angular/animations';
import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

@Component({
  selector: 'app-power-page',
  templateUrl: './power-page.component.html',
  styleUrls: ['./power-page.component.scss']
})
export class PowerPageComponent {
  rosBridgeStatus: string;
  ros: ROSLIB.Ros;
  bmsData: ROSLIB.Topic; //payload type
  errorLogs: string[];
  cellVoltage: number[];
  totalVoltage: number;
  chargingMosfet: number;
  dischargingMosfet: number;

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {

    this.bmsData= new ROSLIB.Topic({
      ros: this.ros,
      name: '/bmsData',
      messageType: 'bms_feedback/BMSData'
    });

    this.bmsData.subscribe((message:any) => {
      console.log(message.cell_voltages);
      this.cellVoltage = message.cell_voltages;
      this.totalVoltage = message.total_voltage;
      this.chargingMosfet = message.charging_mosfet;
      this.dischargingMosfet = message.discharging_logs;
      this.errorLogs = message.error_logs;
    });
  }
}