/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot.triggers;

import org.usfirst.frc.team3319.robot.Robot;

import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * This trigger is active if the user is trying to manually move the gripper while the wrist is moving to a setpoint
 */
public class ManualModeGripperWhileMovingToSetpoint extends Trigger {
  @Override
  public boolean get() {
    return ((Robot.gripperWrist.getCurrentCommand() != null && Robot.gripperWrist.getCurrentCommand().getName().equals("MoveGripperToSetpoint")) && (Robot.oi.getManualGripperMotion() != 0));
  }
}
