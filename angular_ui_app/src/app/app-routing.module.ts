import { NgModule } from '@angular/core';
import {RouterModule, Routes} from '@angular/router'
import { DrivePageComponent } from './pages/drive-page/drive-page.component';
import { TestPageComponent } from './pages/drive-page/test-page/test-page.component';


const routes: Routes = [
  {path: 'drive', component: DrivePageComponent},
  {path: 'test', component: TestPageComponent},

];


@NgModule({
  declarations: [],
  imports: [RouterModule.forRoot(routes),
  ],
  exports: [RouterModule]
})
export class AppRoutingModule { }
