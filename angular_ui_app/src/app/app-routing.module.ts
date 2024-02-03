import { NgModule } from '@angular/core';
import {RouterModule, Routes} from '@angular/router'
import { DrivePageComponent } from './pages/drive-page/drive-page.component';


const routes: Routes = [
  {path: 'drive', component: DrivePageComponent},

];


@NgModule({
  declarations: [],
  imports: [RouterModule.forRoot(routes),
  ],
  exports: [RouterModule]
})
export class AppRoutingModule { }
