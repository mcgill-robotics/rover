import { Component, OnInit } from '@angular/core';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.scss']
})
export class AppComponent implements OnInit {
  // defining updateCounter function here -ida
  updateCounter(value: number) {
    console.log('Update Counter: ', value);
  }

  ngOnInit(): void {

  }
}