import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class ScienceService {
  data: any[] = [];
  data1: number[][][] = [[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]]]; //4 cuvettes, 3 types(ph, humidity, geiger), arrays for each
  constructor() { }

  addData(input: any) {
    this.data.push(input);
  }

  storeData(data:number, cuvetteId:number, dataId:number) {
    this.data1[cuvetteId][dataId].push(data);
  }

  getData() {
    return this.data;
  }

  getStoredData() {
    return this.data1;
  }
}
