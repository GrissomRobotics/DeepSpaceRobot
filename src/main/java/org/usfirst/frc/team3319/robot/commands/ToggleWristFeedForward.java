/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot.commands;

import org.usfirst.frc.team3319.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ToggleWristFeedForward extends Command {
  public ToggleWristFeedForward() {
    // changing variable, does not require anything, does not affect hardware state
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.gripperWrist.toggleFeedForward();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
