package org.usfirst.frc.team3319.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team3319.robot.RobotMap;
import org.usfirst.frc.team3319.robot.commands.MoveArmWithJoystick;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Arm extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX motor = RobotMap.arm;
  private Encoder encoder = RobotMap.armEncoder;

  public Arm(double p, double i, double d) {
    super(p,i,d);
    //TODO figure out appropriate tolerance
    setPercentTolerance(0);
    setOutputRange(-RobotMap.ARM_SPEED, RobotMap.ARM_SPEED);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MoveArmWithJoystick());
  }

  public void raise() {
    motor.set(ControlMode.PercentOutput, RobotMap.ARM_SPEED);
  }

  public void raise(double speed) {
    motor.set(ControlMode.PercentOutput, RobotMap.ARM_SPEED*speed);
  }

  public void lower() {
    motor.set(ControlMode.PercentOutput, -RobotMap.ARM_SPEED);
  }

  public void lower(double speed) {
    motor.set(ControlMode.PercentOutput, -RobotMap.ARM_SPEED*speed);
  }

  @Override
  protected double returnPIDInput() {
    return encoder.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm encoder",encoder.get());
  }

  @Override
  protected void usePIDOutput(double output) {
    motor.set(ControlMode.PercentOutput, output);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetEncoder() {
    encoder.reset();
  }

public void setSpeed(double speed) {
  motor.set(ControlMode.PercentOutput, RobotMap.ARM_SPEED*speed);
}
}
