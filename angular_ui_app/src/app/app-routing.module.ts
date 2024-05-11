import { NgModule } from '@angular/core';
import {RouterModule, Routes} from '@angular/router'
import { PowerPageComponent } from './components/power/power-page/power-page.component';
import { DriveComponent } from './components/drive/drive.component';
import { SciencePageComponent } from './pages/science-page/science-page.component';
import { ArmComponent } from './components/arm/arm.component';
import { AntennaComponent } from './components/antenna/antenna.component';
import { DrivePageComponent } from './pages/drive-page/drive-page.component';


const routes: Routes = [
  {path: '', component: DrivePageComponent},
  {path: 'drive-page', component: DrivePageComponent},
  {path: 'science-page', component: SciencePageComponent},
  // {path: 'power', component: PowerPageComponent},
  {path: 'antenna', component: AntennaComponent},
  {path: 'drive', component: DriveComponent},
  {path: 'arm', component: ArmComponent},
];


@NgModule({
  declarations: [],
  imports: [RouterModule.forRoot(routes),
  ],
  exports: [RouterModule]
})
export class AppRoutingModule { }
