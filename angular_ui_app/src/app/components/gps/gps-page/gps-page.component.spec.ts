import { ComponentFixture, TestBed } from '@angular/core/testing';

import { GpsPageComponent } from './gps-page.component';

describe('GpsPageComponent', () => {
  let component: GpsPageComponent;
  let fixture: ComponentFixture<GpsPageComponent>;

  beforeEach(() => {
    TestBed.configureTestingModule({
      declarations: [GpsPageComponent]
    });
    fixture = TestBed.createComponent(GpsPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
