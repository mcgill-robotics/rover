import { ComponentFixture, TestBed } from '@angular/core/testing';

import { SciencePageComponent } from './science-page.component';

describe('SciencePageComponent', () => {
  let component: SciencePageComponent;
  let fixture: ComponentFixture<SciencePageComponent>;

  beforeEach(() => {
    TestBed.configureTestingModule({
      declarations: [SciencePageComponent]
    });
    fixture = TestBed.createComponent(SciencePageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
