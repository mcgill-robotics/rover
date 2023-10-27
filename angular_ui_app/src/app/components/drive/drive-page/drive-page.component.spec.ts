import { ComponentFixture, TestBed } from '@angular/core/testing';

import { DrivePageComponent } from './drive-page.component';

describe('DrivePageComponent', () => {
  let component: DrivePageComponent;
  let fixture: ComponentFixture<DrivePageComponent>;

  beforeEach(() => {
    TestBed.configureTestingModule({
      declarations: [DrivePageComponent]
    });
    fixture = TestBed.createComponent(DrivePageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
