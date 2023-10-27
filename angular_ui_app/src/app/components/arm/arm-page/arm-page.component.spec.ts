import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ArmPageComponent } from './arm-page.component';

describe('ArmPageComponent', () => {
  let component: ArmPageComponent;
  let fixture: ComponentFixture<ArmPageComponent>;

  beforeEach(() => {
    TestBed.configureTestingModule({
      declarations: [ArmPageComponent]
    });
    fixture = TestBed.createComponent(ArmPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
