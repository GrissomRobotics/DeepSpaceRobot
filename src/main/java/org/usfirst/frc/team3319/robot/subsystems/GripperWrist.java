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
import org.usfirst.frc.team3319.robot.commands.MoveGripperWithJoystick;
import org.usfirst.frc.team3319.robot.custom.GripperSetpoint;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class GripperWrist extends PIDSubsystem {

  private VictorSPX wrist = RobotMap.gripperWrist;
  private Encoder wristEncoder = RobotMap.wristEncoder;
  private GripperSetpoint currentSetpoint;
  private DigitalInput lowerLimitSwitch = RobotMap.lowerWristLimitSwitch;
  private DigitalInput upperLimitSwitch = RobotMap.upperWristLimitSwitch;

  public GripperWrist(double p, double i, double d) {
    /*
    wrist.configRemoteFeedbackFilter(0, RemoteSensorSource.CANifier_Quadrature, 0);
    wrist.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    */
    super(p,i,d);
    setOutputRange(-RobotMap.WRIST_SPEED, RobotMap.WRIST_SPEED);
    //TODO find the tolerance
    setPercentTolerance(0.0);
    //upon initialization, the gripper should be fully folded back
    currentSetpoint = GripperSetpoint.FullyFolded;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MoveGripperWithJoystick());
  }

  @Override 
  public void periodic() {
    SmartDashboard.putNumber("Wrist encoder", wristEncoder.get());
    SmartDashboard.putBoolean("Lower wrist limit switch", lowerLimitSwitch.get());
    SmartDashboard.putBoolean("Upper wrist limit switch", upperLimitSwitch.get());

  }

  public void raise() {
    setSpeed(RobotMap.WRIST_SPEED);
  }

  public void raise(double speed) {
    setSpeed(speed*RobotMap.WRIST_SPEED);
  }

  public void lower() {
    setSpeed(-RobotMap.WRIST_SPEED);
  }

  public void lower(double speed) {
    setSpeed(speed*-RobotMap.WRIST_SPEED);
  }

  public void stopWrist() {
    wrist.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  protected double returnPIDInput() {
    return wristEncoder.get();
  }

  @Override
  protected void usePIDOutput(double output) {
    setSpeed(output);
  }

  public void resetEncoder() {
    wristEncoder.reset();
  }

  public void setCurrentSetpoint(GripperSetpoint pos) {
    currentSetpoint = pos;
  }

  public GripperSetpoint getCurrentSetpoint() {
    return currentSetpoint;
  }

  /**
   * This method should be used for all applications involving changing speed in the gripper,
   * as it contains the limit switch logic that prevents the subsystem from harming itself
   * @param speed the speed to set the wrist with, [-1,1]
   */
  public void setSpeed(double speed) {
    //TODO someone other than Jack look at this for consistency and logic
    if (!lowerLimitSwitch.get() && !upperLimitSwitch.get()) {
      //if neither limit switch has been triggered, it is safe to just set the speed
      wrist.set(ControlMode.PercentOutput, speed*RobotMap.WRIST_SPEED);
    } else if (lowerLimitSwitch.get()) {
      if (speed < 0)
        //if the requested speed is less than zero (lowering) and the lower limit switch is triggered,
        //disallow
        wrist.set(ControlMode.PercentOutput, 0);
      else
        wrist.set(ControlMode.PercentOutput, speed*RobotMap.WRIST_SPEED);
    } else if (upperLimitSwitch.get()) {
      if (speed>0)
        //if the requested speed is greater than zero (raising) and the upper limit switch is triggered,
        //disallow
        wrist.set(ControlMode.PercentOutput, 0);
      else
        wrist.set(ControlMode.PercentOutput, speed*RobotMap.WRIST_SPEED);
    }
  }
}
