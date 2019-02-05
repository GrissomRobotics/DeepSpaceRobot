/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot.triggers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * This trigger allows the POV pad on a given controller to be used like a button
 * for the top part of it
 */
public class BottomPOVButton extends Trigger {

  private Joystick stick;

  /**
   * 
   * @param stick the joystick that this trigger is affiliated with
   */
  public BottomPOVButton(Joystick stick) {
    super();
    this.stick = stick;
  }


  @Override
  public boolean get() {
    //180 is the top on the POV
    return stick.getPOV()==180;
  }
}
