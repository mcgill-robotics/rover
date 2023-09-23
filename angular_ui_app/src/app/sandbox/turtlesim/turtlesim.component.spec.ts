import { ComponentFixture, TestBed } from '@angular/core/testing';

import { TurtlesimComponent } from './turtlesim.component';

describe('TurtlesimComponent', () => {
  let component: TurtlesimComponent;
  let fixture: ComponentFixture<TurtlesimComponent>;

  beforeEach(() => {
    TestBed.configureTestingModule({
      declarations: [TurtlesimComponent]
    });
    fixture = TestBed.createComponent(TurtlesimComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
