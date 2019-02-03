/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot.commands;

import org.usfirst.frc.team3319.robot.Robot;
import org.usfirst.frc.team3319.robot.custom.ArmSetpoint;

import edu.wpi.first.wpilibj.command.Command;

public class RaiseArmToNextSetpoint extends Command {

  private Command armCommand;

  public RaiseArmToNextSetpoint() {
    //Does not require anything, the command it starts is responsible for filing requirements
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    ArmSetpoint setpoint = Robot.arm.getCurrentSetpoint();
    armCommand = new MoveArmToSetpoint(ArmSetpoint.getPrevious(setpoint));
    armCommand.start();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return armCommand.isCompleted();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    armCommand.cancel();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}