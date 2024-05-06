import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class ScienceService {
  data: any[] = [];
  constructor() { }

  addData(input: any) {
    this.data.push(input);
  }

  getData() {
    return this.data;
  }
}
