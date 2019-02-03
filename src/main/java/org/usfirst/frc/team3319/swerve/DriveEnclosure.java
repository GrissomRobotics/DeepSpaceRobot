package org.usfirst.frc.team3319.swerve;

import org.usfirst.frc.team3319.robot.RobotMap;
import org.usfirst.frc.team3319.robot.custom.EncoderEnabledMotor;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class DriveEnclosure extends BaseEnclosure {

	private SpeedController driveMotor;
	private SpeedController steerMotor;
	
	private EncoderEnabledMotor pidSystem;
	
	private Encoder encoder;
	
	private boolean reverseEncoder = false;
	private boolean reverseSteer = false;

    public DriveEnclosure(String name, SpeedController driveMotor, SpeedController steerMotor, double gearRatio, Encoder encoder, double tolerance) {

        super(name, gearRatio);
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.encoder = encoder;
        this.pidSystem = new EncoderEnabledMotor(RobotMap.DRIVETRAIN_P,
        										 RobotMap.DRIVETRAIN_I,
        										 RobotMap.DRIVETRAIN_D,
        										 steerMotor,
        										 encoder,
        										 tolerance
        										);
        
    }

    @Override
    public void stop() {
        this.steerMotor.stopMotor();
        this.driveMotor.stopMotor();
    }

    @Override
    public void setSpeed(double speed) {
		driveMotor.set(RobotMap.MAX_DRIVE_SPEED*speed);
    }

    @Override
    public void setAngle(double angle) {
    	//steerMotor.set(ControlMode.Position, (reverseSteer ? -1 : 1) * angle * gearRatio);
		pidSystem.setSetpoint((reverseSteer ? -1 : 1 * angle * gearRatio));//gearRatio is the counts per revolution
    	pidSystem.enable();
    }

    @Override
    public int getEncPosition() {
        int reverse = reverseEncoder ? -1 : 1;
        return reverse * encoder.get();
    }	

    @Override
    public void setEncPosition(int position) {
    }

    public SpeedController getDriveMotor()
	{
		return driveMotor;
	}
	
	public SpeedController getSteerMotor()
	{
		return steerMotor;
	}
	
	public boolean isReverseEncoder()
	{
		return reverseEncoder;
	}
	
	public void setReverseEncoder(boolean reverseEncoder)
	{
		this.reverseEncoder = reverseEncoder;
	}
	
	public void setReverseSteerMotor(boolean reverseSteer)
	{
		this.reverseSteer = reverseSteer;
	}
	
	public void disable() {
		this.pidSystem.getPIDController().disable();
	}

	public void setPIDValuesFromDashboard() {
		this.pidSystem.getPIDController().setP(RobotMap.DRIVETRAIN_P);//SmartDashboard.getNumber("Proportional", 0.0));
		this.pidSystem.getPIDController().setI(RobotMap.DRIVETRAIN_I);//SmartDashboard.getNumber("Integral", 0.0));
		this.pidSystem.getPIDController().setD(RobotMap.DRIVETRAIN_D);//SmartDashboard.getNumber("Differential", 0.0));
	}
}