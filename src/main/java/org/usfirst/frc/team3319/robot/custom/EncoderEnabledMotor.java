package org.usfirst.frc.team3319.robot.custom;

import org.usfirst.frc.team3319.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class EncoderEnabledMotor extends PIDSubsystem {

	SpeedController motor;
	Encoder encoder;
	

	public EncoderEnabledMotor(double p, double i, double d, SpeedController motor, Encoder encoder,double tolerance) {
		super(p, i, d);
		this.motor = motor;
		this.encoder = encoder;
		this.getPIDController().setOutputRange(-RobotMap.TURN_SPEED, RobotMap.TURN_SPEED);
		this.getPIDController().setPercentTolerance(tolerance);
	}

	@Override
	protected double returnPIDInput() {
		return encoder.get();
	}

	@Override
	protected void usePIDOutput(double output) {
		motor.set(output);
	}

	@Override
	protected void initDefaultCommand() {
	}

	public void enable() {
		this.getPIDController().enable();
	}
	
	public void disable() {
		this.getPIDController().disable();
	}
	
	public boolean isOnTarget() {
		if (this.getPIDController().onTarget()) {
			disable();
		}
		return this.getPIDController().onTarget();
	}
	
	@Override
	public void periodic() {
		if (getPIDController().isEnabled()) {
			isOnTarget();
		}
		/*
		double p = SmartDashboard.getNumber("Proportional", 0.0);
		double i = SmartDashboard.getNumber("Integral", 0.0);
		double d = SmartDashboard.getNumber("Differential", 0.0);
		this.getPIDController().setP(p);
		this.getPIDController().setI(i);
		this.getPIDController().setD(d);
		*/
		
	}
}
