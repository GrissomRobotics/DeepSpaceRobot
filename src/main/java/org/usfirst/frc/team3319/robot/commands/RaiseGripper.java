/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot.commands;

import org.usfirst.frc.team3319.robot.Robot;
import org.usfirst.frc.team3319.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class RaiseGripper extends Command {
  private double speed;

  public RaiseGripper() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    //requires(Robot.gripperWrist);
    this.speed = RobotMap.WRIST_SPEED;
  }

  public RaiseGripper(double speed) {
    this();
    this.speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.gripperWrist.raise(speed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.gripperWrist.stopWrist();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}