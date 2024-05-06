import { Component } from '@angular/core';
import { ScienceService } from 'src/app/service/science.service';

@Component({
  selector: 'app-science-page',
  templateUrl: './science-page.component.html',
  styleUrls: ['./science-page.component.scss']
})
export class SciencePageComponent {

  constructor(private scienceService: ScienceService) {

  }
  data: string;

  collect() {
    this.data = this.scienceService.getData().toString();
  }
  add() {
    this.scienceService.addData("halo");
  }
}
