/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.usfirst.frc.team3319.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class GripperWheels extends Subsystem {
  private VictorSPX gripperWheels = RobotMap.gripperWheels;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void intakeCargo(){
    gripperWheels.set(ControlMode.PercentOutput, RobotMap.GRIPPER_WHEELS_SPEED);
  }
  public void expelCargo(){
    gripperWheels.set(ControlMode.PercentOutput, -RobotMap.GRIPPER_WHEELS_SPEED);
  }
  public void stopWheels(){
    gripperWheels.set(ControlMode.PercentOutput, 0.0);
  }
}
