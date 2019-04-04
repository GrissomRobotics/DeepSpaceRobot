/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot.subsystems;

import org.usfirst.frc.team3319.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid frontSolenoid = RobotMap.frontClimber;
  private DoubleSolenoid backSolenoid = RobotMap.backClimber;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void extendBack() {
    backSolenoid.set(Value.kForward);
  }

  public void retractBack() {
    backSolenoid.set(Value.kReverse);
  }

  public void extendFront() {
    frontSolenoid.set(Value.kForward);
  }

  public void retractFront() {
    frontSolenoid.set(Value.kReverse);
  }

  public void retractAll() {
    retractFront();
    retractBack();
  }


}
