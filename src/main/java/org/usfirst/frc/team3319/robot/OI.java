/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot;

import org.usfirst.frc.team3319.robot.commands.*;

import org.usfirst.frc.team3319.robot.custom.Controller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	
	Controller driveController;
	Controller otherController;

	public OI() {
		driveController = new Controller(0);
		otherController = new Controller(1);
		SmartDashboard.putBoolean("FEILD Centric", false);

		otherController.getAButton().whenPressed(new LowerGripperToNextSetpoint());
		otherController.getYButton().whenPressed(new RaiseGripperToNextSetpoint());
		otherController.getRightBumper().whileHeld(new ExpelCargo());
		otherController.getLeftBumper().whileHeld(new IntakeCargo());
		otherController.getBButton().whenPressed(new ExtendFinger());
		otherController.getXButton().whenPressed(new RetractFinger());
		otherController.getTopPOVButton().whenActive(new RaiseArmToNextSetpoint());;
		otherController.getBottomPOVButton().whenActive(new LowerArmToNextSetpoint());
		}

	public double getForward() {
		return driveController.getLeftJoystickYAxis();
	}
	public double getStrafe() {
		//negate because the swerve drive configuration is inverted
		return driveController.getLeftJoystickXAxis();
	}
	
	public double getClockwiseRotation() {
		//the reason this subtraction is used is that axis 3 is the right trigger(clockwise rotation) and axis 2 is the left trigger(counterclockwise rotation)
		//each only goes from 0 to 1, so subtraction is necessary to get the correct value with the negative
		return driveController.getRightTrigger()-driveController.getLeftTrigger();
	}

	public double getManualArmMotion() {
		//negate because forward is negative
		return -otherController.getLeftJoystickYAxis();
	}

	public double getManualGripperMotion() {
		//negate this value because forward is negative
		return -otherController.getRightJoystickYAxis();
	}
}
