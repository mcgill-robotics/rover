import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import {RouterModule, Routes} from '@angular/router'
import { TurtlesimComponent } from './sandbox/turtlesim/turtlesim.component';
import { PowerPageComponent } from './components/power/power-page/power-page.component';
import { DrivePageComponent } from './components/drive/drive-page/drive-page.component';
import { SciencePageComponent } from './components/science/science-page/science-page.component';
import { ArmPageComponent } from './components/arm/arm-page/arm-page.component';
import { GpsPageComponent } from './components/gps/gps-page/gps-page.component';
import { TestPageComponent } from './sandbox/test-page/test-page.component';


const routes: Routes = [
  {path: 'turtlesim', component: TurtlesimComponent},
  {path: 'power', component: PowerPageComponent},
  {path: 'drive', component: DrivePageComponent},
  {path: 'science', component: SciencePageComponent},
  {path: 'arm', component: ArmPageComponent},
  {path: 'gps', component: GpsPageComponent},
  {path: 'test', component: TestPageComponent},


  
];


@NgModule({
  declarations: [],
  imports: [RouterModule.forRoot(routes),
  ],
  exports: [RouterModule]
})
export class AppRoutingModule { }
