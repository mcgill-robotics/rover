import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import {RouterModule, Routes} from '@angular/router'
import { PowerPageComponent } from './components/power/power-page/power-page.component';
import { DriveComponent } from './components/drive/drive.component';
import { SciencePageComponent } from './components/science/science-page/science-page.component';
import { ArmComponent } from './components/arm/arm.component';
import { AntennaComponent } from './components/antenna/antenna.component';


const routes: Routes = [
  {path: 'antenna', component: AntennaComponent},
  {path: 'power', component: PowerPageComponent},
  {path: 'drive', component: DriveComponent},
  {path: 'science', component: SciencePageComponent},
  {path: 'arm', component: ArmComponent},
];


@NgModule({
  declarations: [],
  imports: [RouterModule.forRoot(routes),
  ],
  exports: [RouterModule]
})
export class AppRoutingModule { }
