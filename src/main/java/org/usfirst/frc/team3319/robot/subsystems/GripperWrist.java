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
  private DigitalInput limitSwitch = RobotMap.wristLimitSwitch;

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
    SmartDashboard.putBoolean("Wrist limit switch", limitSwitch.get());
  }

  public void raise() {
    wrist.set(ControlMode.PercentOutput, RobotMap.WRIST_SPEED);
  }

  public void raise(double speed) {
    wrist.set(ControlMode.PercentOutput, speed*RobotMap.WRIST_SPEED);
  }

  public void lower() {
    wrist.set(ControlMode.PercentOutput, -RobotMap.WRIST_SPEED);
  }

  public void lower(double speed) {
    wrist.set(ControlMode.PercentOutput, speed*-RobotMap.WRIST_SPEED);
  }

  public void stopWrist() {
    wrist.set(ControlMode.PercentOutput, 0.0);
  }

  public void stopAll() {
    stopWrist();
  }

  @Override
  protected double returnPIDInput() {
    return wristEncoder.get();
  }

  @Override
  protected void usePIDOutput(double output) {
    wrist.set(ControlMode.PercentOutput, output);
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

public void setSpeed(double speed) {
  wrist.set(ControlMode.PercentOutput, speed*RobotMap.WRIST_SPEED);
}
}
