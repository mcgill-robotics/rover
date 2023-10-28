import { Component } from '@angular/core';
// import { data } from 'jquery';
// import * as $ from "jquery";
// import * as jQuery from 'jquery';


import * as $ from 'jquery';

// window['$'] = window['jQuery'] = $;



@Component({
  selector: 'app-button1',
  templateUrl: './button1.component.html',
  styleUrls: ['./button1.component.scss']
})
export class Button1Component {
  // make ajax request?
  counter: number = 0;

  increment() {
    console.log("calling ajax")
  
    $.ajax({
      type: "POST",
      url: "count.py",
      data: {param: this.counter}
    }).done(function(m) {
      console.log("return of ajax")
      console.log(m);
    })
  }
  


}
