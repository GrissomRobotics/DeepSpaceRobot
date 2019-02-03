/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot.commands;

import org.usfirst.frc.team3319.robot.*;

import org.usfirst.frc.team3319.robot.custom.GripperSetpoint;

import edu.wpi.first.wpilibj.command.Command;

public class LowerGripperToNextSetpoint extends Command {

  private Command gripperCommand;

  public LowerGripperToNextSetpoint() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    GripperSetpoint setpoint = Robot.gripperWrist.getCurrentSetpoint();
    gripperCommand = new MoveGripperToSetpoint(GripperSetpoint.getPrevious(setpoint));
    gripperCommand.start();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return gripperCommand.isCompleted();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    gripperCommand.cancel();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
