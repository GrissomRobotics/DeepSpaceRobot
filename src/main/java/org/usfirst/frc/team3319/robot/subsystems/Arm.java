package org.usfirst.frc.team3319.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team3319.robot.RobotMap;
import org.usfirst.frc.team3319.robot.commands.MoveArmWithJoystick;
import org.usfirst.frc.team3319.robot.custom.ArmSetpoint;

import edu.wpi.first.wpilibj.DigitalInput;
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
  private ArmSetpoint currentSetpoint;
  private DigitalInput lowerLimitSwitch = RobotMap.lowerArmLimitSwitch;
  private DigitalInput upperLimitSwitch = RobotMap.upperArmLimitSwitch;

  public Arm(double p, double i, double d, double f) {
    super(p,i,d,f,RobotMap.PID_PERIOD);
    //TODO figure out appropriate tolerance for the arm
    setPercentTolerance(3.0);
    setOutputRange(-RobotMap.ARM_SPEED_RAISE, RobotMap.ARM_SPEED_RAISE);
    currentSetpoint = ArmSetpoint.BeginningConfiguration;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MoveArmWithJoystick());
  }

  public void raise() {
    setSpeed(1.0);
  }

  public void raise(double speed) {
    setSpeed(speed);
  }

  public void lower() {
    setSpeed(-1.0);
  }

  public void lower(double speed) {
    setSpeed(-speed);
  }

  @Override
  protected double returnPIDInput() {
    return encoder.get();
  }

  @Override
  protected void usePIDOutput(double output) {
    setSpeed(output);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm encoder", encoder.get());
    SmartDashboard.putBoolean("Lower arm limit switch", lowerLimitSwitch.get());
    SmartDashboard.putBoolean("Upper arm limit switch", upperLimitSwitch.get());
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetEncoder() {
    encoder.reset();
  }

  
  /**
   * This method should be used for all applications involving changing speed in the gripper,
   * as it contains the limit switch logic that prevents the subsystem from harming itself
   * This method scales the requested speed, so it should not be called with arguments that have
   * already been scaled.
   * @param speed the speed to set the arm with, [-1,1]
   */
  public void setSpeed(double speed) {
    if (!lowerLimitSwitch.get() && !upperLimitSwitch.get()) {
      //if neither limit switch has been triggered, it is safe to just set the speed
      motor.set(ControlMode.PercentOutput, speed*((speed>0)?RobotMap.ARM_SPEED_RAISE:RobotMap.ARM_SPEED_LOWER));
    } else if (lowerLimitSwitch.get()) {
      if (speed < 0)
        //if the requested speed is less than zero (lowering) and the lower limit switch is triggered,
        //disallow
        motor.set(ControlMode.PercentOutput, 0);
      else
        motor.set(ControlMode.PercentOutput, speed*RobotMap.ARM_SPEED_LOWER);
    } else if (upperLimitSwitch.get()) {
      if (speed>0)
        //if the requested speed is greater than zero (raising) and the upper limit switch is triggered,
        //disallow
        motor.set(ControlMode.PercentOutput, 0);
      else
        motor.set(ControlMode.PercentOutput, speed*RobotMap.ARM_SPEED_RAISE);
    }
  }

  public void setCurrentSetpoint(ArmSetpoint pos) {
    currentSetpoint = pos;
  }

  public ArmSetpoint getCurrentSetpoint() {
    return currentSetpoint;
  }
}
