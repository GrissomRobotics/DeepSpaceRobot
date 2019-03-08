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
public class Finger extends Subsystem {

  private DoubleSolenoid fingerSolenoid = RobotMap.finger; 
  //private Compressor compressor = RobotMap.compressor;

  @Override
  public void initDefaultCommand() {
  }

  @Override
  public void periodic() {
   /* if (SmartDashboard.getBoolean("Enable compressor? ", true)) {
      enableCompressor();  
    } else {
      disableCompressor();
    }
    */
  }

  public void extend() {
    fingerSolenoid.set(Value.kForward);
  }

  public void retract() {
    fingerSolenoid.set(Value.kReverse);
  }

  public void toggle() {
    switch(fingerSolenoid.get()) {
      case kForward:
        fingerSolenoid.set(Value.kReverse);
      case kReverse:
        fingerSolenoid.set(Value.kForward);
      case kOff:
        //
    }
  }

  /*
  public void disableCompressor() {
    compressor.stop();
  }

  public void enableCompressor() {
    compressor.start();
  }
  */
}
