/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot.commands;

import org.usfirst.frc.team3319.robot.Robot;
import org.usfirst.frc.team3319.robot.RobotMap;
import org.usfirst.frc.team3319.robot.custom.GripperSetpoint;

import edu.wpi.first.wpilibj.command.Command;

public class MoveGripperToSetpoint extends Command {
  private int setpoint;
  private GripperSetpoint desiredPos;

  public MoveGripperToSetpoint(GripperSetpoint setpoint) {
    requires(Robot.gripperWrist);
    desiredPos = setpoint;
    switch (setpoint) {
      case FullyFolded: this.setpoint = RobotMap.FULLY_FOLDED_WRIST_SETPOINT;
      case Expel: this.setpoint = RobotMap.EXPEL_WRIST_SETPOINT;
      case Intake: this.setpoint = RobotMap.INTAKE_WRIST_SETPOINT;
    }

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.gripperWrist.setSetpoint(setpoint);
    Robot.gripperWrist.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.gripperWrist.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.gripperWrist.disable();
    if (Robot.gripperWrist.onTarget()) {
      //if we are at the right place, update the gripper's internal variables
      Robot.gripperWrist.setCurrentSetpoint(desiredPos);
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
