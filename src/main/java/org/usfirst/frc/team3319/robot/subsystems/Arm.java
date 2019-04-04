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
  private DigitalInput limitSwitch = RobotMap.lowerArmLimitSwitch;

  public Arm(double p, double i, double d, double f) {
    super(p,i,d,f,RobotMap.PID_PERIOD);
    //TODO figure out appropriate tolerance for the arm
    setPercentTolerance(3.0);
    setOutputRange(-0.4, 0.4);
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
    SmartDashboard.putBoolean("Arm limit switch", limitSwitch.get());
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
    SmartDashboard.putNumber("Requested arm speed ", speed);

    /*
    if (limitSwitch.get()) {
      if (speed == 0) {
        motor.set(ControlMode.PercentOutput, 0);
      }
      else {
        if (speed>0) {
          //if we are trying to raise and the switch is depressed, we are fine
          motor.set(ControlMode.PercentOutput, speed*RobotMap.ARM_SPEED_RAISE);
        } else {
          //if the limit switch is depressed and the speed is requested is less than than 0 (i.e. lowering)
          motor.set(ControlMode.PercentOutput, 0);
        }
      }
    }
    else { //if the limit switch hasn't been depressed
    */
      if (speed>0) {
        motor.set(ControlMode.PercentOutput, speed*RobotMap.ARM_SPEED_RAISE);
      } else if (speed<0){
        motor.set(ControlMode.PercentOutput, speed*RobotMap.ARM_SPEED_LOWER);
      } else {
        //motor.set(ControlMode.PercentOutput, RobotMap.ARM_HOLD_SPEED);//RobotMap.ARM_HOLD_SPEED);
      }
    //}
  }

  private double calculateHoldSpeed() {
    int encoderVal = encoder.get();
    double guess = -7.5629683596753e-4*encoderVal*encoderVal + 0.05439389866302*encoderVal - 0.7833584316;
    if (guess<0) {
      return 0.0;
    }
    return guess+0.1;
  }

  public void setCurrentSetpoint(ArmSetpoint pos) {
    currentSetpoint = pos;
  }

  public ArmSetpoint getCurrentSetpoint() {
    return currentSetpoint;
  }
}