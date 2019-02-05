package org.usfirst.frc.team3319.robot.custom;

import org.usfirst.frc.team3319.robot.triggers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

//This class provides the interface for the XBox Controller or the Logitech Controller, so that the OI class is not responsible for determining logic 
//like calculating the appropriate rotation value or deadzoning the input
//Additionally, this provides greater flexibility for changing to a new controller, as simply a new class would be created that 
//associated each button with the appropriate number from the driver station, rather than the OI class being responsible for it
//The primary purpose of this class is to abstract away controller level logic from the main program

public class Controller {
    private final static double CONTROLLER_DEAD_ZONE = 0.05;
    private Joystick stick;
    private JoystickButton Y;
    private JoystickButton A;
    private JoystickButton B;
    private JoystickButton X;
    private JoystickButton rightBumper;
    private JoystickButton leftBumper;
    private Trigger topPOV;
    private Trigger bottomPOV;

    /**
     * 
     * @param channel the port that controller is plugged into, as determined on the driver station
     */
    public Controller(int channel) {
        stick = new Joystick(channel);
        Y = new JoystickButton(stick, 4);
        A = new JoystickButton(stick, 1);
        B = new JoystickButton(stick, 2);
        X = new JoystickButton(stick, 3);
        topPOV = new TopPOVButton(stick);
        bottomPOV = new BottomPOVButton(stick);
        rightBumper = new JoystickButton(stick,6);
        leftBumper = new JoystickButton(stick, 5);
    }

    private double valueIfWithinDeadZone(double value) {
		if (Math.abs(value)<CONTROLLER_DEAD_ZONE) {
			return 0.0;
		}
		return value;
	}
	
	public double getLeftJoystickYAxis() {
		return valueIfWithinDeadZone(stick.getRawAxis(1));
	}
	public double getLeftJoystickXAxis() {
		return valueIfWithinDeadZone(stick.getRawAxis(0));
    }

    public double getRightJoystickYAxis() {
		return valueIfWithinDeadZone(stick.getRawAxis(5));
	}
	public double getRightJoystickXAxis() {
		return valueIfWithinDeadZone(stick.getRawAxis(4));
    }
    
    public double getLeftTrigger() {
        return valueIfWithinDeadZone(stick.getRawAxis(2));
    }

    public double getRightTrigger() {
        return valueIfWithinDeadZone(stick.getRawAxis(3));
    }

    public JoystickButton getYButton() {
        return this.Y;
    }

    public JoystickButton getAButton() {
        return this.A;
    }

    public JoystickButton getXButton() {
        return this.X;
    }

    public JoystickButton getBButton() {
        return this.B;
    }

    public Trigger getTopPOVButton() {
        return this.topPOV;
    }

    public Trigger getBottomPOVButton() {
        return this.bottomPOV;
    }


    public JoystickButton getRightBumper(){
        return this.rightBumper;
    }
    public JoystickButton getLeftBumper(){
        return this.leftBumper;
    }
}