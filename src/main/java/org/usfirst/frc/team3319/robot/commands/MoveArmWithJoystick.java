/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot.commands;

import org.usfirst.frc.team3319.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class MoveArmWithJoystick extends Command {
  private Command gripperCommand;

  public MoveArmWithJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /*
    TODO decide if this logic is necessary during manual control for now it is disabled because the gripper is not hooked up
    if (Robot.oi.getManualArmMotion() > 0) {
      if (Robot.gripperWrist.getCurrentSetpoint() == GripperSetpoint.Expel) {
            Robot.arm.setSpeed(Robot.oi.getManualArmMotion());
      }
      else {
        gripperCommand = new MoveGripperToSetpoint(GripperSetpoint.Expel);
        gripperCommand.start();
     }
    }
    */
    Robot.arm.setSpeed(Robot.oi.getManualArmMotion());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.arm.stop();
    if (gripperCommand !=null) {
      gripperCommand.cancel();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
