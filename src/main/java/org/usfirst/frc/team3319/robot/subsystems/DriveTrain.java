package org.usfirst.frc.team3319.robot.subsystems;

import org.usfirst.frc.team3319.robot.RobotMap;
import org.usfirst.frc.team3319.robot.commands.DriveWithJoystick;
import org.usfirst.frc.team3319.swerve.*;
import org.usfirst.frc.team3319.swerve.math.*;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {
	
	private SwerveDrive swerve = RobotMap.swerve;
	private PigeonIMU gyro = RobotMap.gyro;

	//For my purposes, the drive train owns the line following functionality of the camera
	private boolean followLine = true;
	private CentricMode mode = CentricMode.ROBOT;
	private double driveSpeed = RobotMap.DRIVE_SPEED;
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	setDefaultCommand(new DriveWithJoystick());
    }
    
    public void move(double forward, double strafe,double clockwiserotation) {
    	swerve.move(forward, strafe, clockwiserotation, Double.valueOf(gyro.getFusedHeading()));
    }
    
    public void setCentricMode(CentricMode mode) {
			swerve.setCentricMode(mode);
			this.mode = mode;
			if (mode==CentricMode.FIELD) {
				SmartDashboard.putBoolean("FIELD Centric", true);
			} else {
				SmartDashboard.putBoolean("FIELD Centric", false);
			}
    }
    
    public void periodic() {
		/*
    	SmartDashboard.putNumber("Back Left Encoder", RobotMap.backLeftEncoder.get());
    	SmartDashboard.putNumber("Front Left Encoder", RobotMap.frontLeftEncoder.get());
    	SmartDashboard.putNumber("Back Right Encoder", RobotMap.backRightEncoder.get());
		SmartDashboard.putNumber("Front Right Encoder", RobotMap.frontRightEncoder.get());
		*/
		
		SmartDashboard.putNumber("Gyro", gyro.getFusedHeading());
		//SmartDashboard.putString("Ultrasonic reading", RobotMap.ultra.readLastRange().substring(1));
    }
    
    public void resetEncoders() {
    	RobotMap.backLeftEncoder.reset();
    	RobotMap.backRightEncoder.reset();
			RobotMap.frontLeftEncoder.reset();
    	RobotMap.frontRightEncoder.reset();
    }
    
    public void stop() {
    	swerve.stop();
    }
    
    public void disableTurnPID() {
    	swerve.disablePID();
    }

	public void setPIDValuesFromDashboard() {
		swerve.setPIDValuesFromDashboard();
	}
	
	public void resetGyro() {
		gyro.setFusedHeading(0);
	}

	public void setSteerAngle(double angle) {
		swerve.setSteerPosition(angle);
	}
	
	public void toggleLineFollowing() {
		this.followLine = !this.followLine;
	}

	public boolean getDisplayLine() {
		return this.followLine;
	}

	public void resetWheelPosition() {
		this.swerve.resetWheelPosition();
	}

	public void toggleCentricMode() {
		switch (this.mode) {
			case FIELD:
				this.mode = CentricMode.ROBOT;
			case ROBOT:
				this.mode = CentricMode.FIELD;
		}
		this.setCentricMode(this.mode);
	}
	
	public void toggleSpeed() {
		switch(this.driveSpeed) {
			case RobotMap.DRIVE_SPEED:
				this.driveSpeed = RobotMap.REDUCED_SPEED;
			case RobotMap.REDUCED_SPEED:
				this.driveSpeed = RobotMap.DRIVE_SPEED;
		}
	}
		
}

