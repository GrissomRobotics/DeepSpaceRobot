/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot.commands;

import org.usfirst.frc.team3319.robot.Robot;
import org.usfirst.frc.team3319.robot.RobotMap;
import org.usfirst.frc.team3319.robot.custom.ArmSetpoint;
import org.usfirst.frc.team3319.robot.custom.GripperSetpoint;

import edu.wpi.first.wpilibj.command.Command;

public class MoveArmToSetpoint extends Command {
  private int setpoint;
  private ArmSetpoint desiredPos;

  private Command gripperCommand;

  public MoveArmToSetpoint(ArmSetpoint setpoint) {
    requires(Robot.arm);
    desiredPos = setpoint;
    switch (setpoint) {
      case BeginningConfiguration: this.setpoint = RobotMap.INITIAL_ARM_POSITION;
      case BottomHatch: this.setpoint = RobotMap.BOTTOM_HATCH_ARM_SETPOINT;
      case BottomPort: this.setpoint = RobotMap.BOTTOM_PORT_ARM_SETPOINT;
      case MiddleHatch: this.setpoint = RobotMap.MIDDLE_HATCH_ARM_SETPOINT;
      case MiddlePort: this.setpoint = RobotMap.MIDDLE_PORT_ARM_SETPOINT;
      case HighHatch: this.setpoint = RobotMap.HIGH_HATCH_ARM_SETPOINT;
      case HighPort: this.setpoint = RobotMap.HIGH_PORT_ARM_SETPOINT;
    }

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //TODO this might not be the correct placement for this code, (maybe it should go it execute)
    //but the arm subsystem should stop motion in the event that the arm is entering a dangerous configuration
    if (Robot.gripperWrist.getCurrentSetpoint() != GripperSetpoint.Expel) {
      gripperCommand = new MoveGripperToSetpoint(GripperSetpoint.Expel);
      gripperCommand.start();
    }

    Robot.arm.setSetpoint(setpoint);
    Robot.arm.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.arm.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //TODO I don't know if disabling is the desired behavior here, holding in place might be preferable
    Robot.arm.disable();
    if (Robot.arm.onTarget()) {
      //if we are at the right place, update the arm's internal variables
      Robot.arm.setCurrentSetpoint(desiredPos);
    }
    if (gripperCommand != null) {
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
