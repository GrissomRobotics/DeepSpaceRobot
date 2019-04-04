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

import edu.wpi.first.wpilibj.Counter;
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
  private DigitalInput limitSwitch = RobotMap.wristLimitSwitch;
  private GripperSetpoint currentSetpoint;
  private boolean useLimitSwitch = true;
  private boolean useFeedForward = true;

  public GripperWrist(double p, double i, double d) {
    super(p,i,d);
    setOutputRange(-1, 1);
    setPercentTolerance(3.0);
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
    SmartDashboard.putBoolean("Using wrist limit switch", useLimitSwitch);

    if (limitSwitch.get()) {
     // resetEncoder();
    }
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
    setSpeed(speed);
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

  public void toggleUsingLimitSwitch() {
    useLimitSwitch = !useLimitSwitch;
  }

  public void toggleFeedForward() {
    useFeedForward = !useFeedForward;
  }

  /**
   * This method should be used for all applications involving changing speed in the gripper,
   * as it contains the limit switch logic that prevents the subsystem from harming itself
   * This method scales the requested speed, so it should not be called with arguments that have
   * already been scaled.
   * @param speed the speed to set the wrist with, [-1,1]
   */
  public void setSpeed(double speed) {
    if (useFeedForward) {
      if (useLimitSwitch) {
        if (limitSwitch.get()) {
            if (speed == 0) {
              wrist.set(ControlMode.PercentOutput, 0);
            }
            else {
              if (speed<0) {
                //if we are trying to lower and the switch is depressed, we are fine
                wrist.set(ControlMode.PercentOutput, speed*RobotMap.WRIST_SPEED);
              } else {
                //if the limit switch is depressed and the speed is requested is greater than 0 (i.e. raising)
                wrist.set(ControlMode.PercentOutput, 0);
              }
            }
          }
        else { //if the limit switch hasn't been depressed
          if (speed ==0) {
            wrist.set(ControlMode.PercentOutput, RobotMap.WRIST_HOLD_SPEED);
          } else {
            wrist.set(ControlMode.PercentOutput, speed*RobotMap.WRIST_SPEED); 
          }
        }
      } else { //if we aren't using the limit switch the reason for this is that so if the limit switch fails we are not dead in the water
        if (speed ==0) {
          wrist.set(ControlMode.PercentOutput, RobotMap.WRIST_HOLD_SPEED);
        } else {
          wrist.set(ControlMode.PercentOutput, speed*RobotMap.WRIST_SPEED); 
        }
      }
    } else {
      wrist.set(ControlMode.PercentOutput, speed*RobotMap.WRIST_SPEED);
    }
  }
}
