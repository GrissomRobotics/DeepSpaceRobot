/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.usfirst.frc.team3319.robot.custom.UltrasonicSensor;
import org.usfirst.frc.team3319.swerve.DriveEnclosure;
import org.usfirst.frc.team3319.swerve.SwerveDrive;
import org.usfirst.frc.team3319.swerve.SwerveEnclosure;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	//Robot variables
	public final static double LENGTH = 21;
	public final static double WIDTH = 20.5;
	public final static double TURN_SPEED = 1;
	public final static double MAX_DRIVE_SPEED = 0.5;
	public final static double WRIST_SPEED = 1.00;
	public final static double WRIST_HOLD_SPEED = 0.2;
	public final static double GRIPPER_WHEELS_SPEED = 0.5;
	public final static double ARM_SPEED_RAISE = 1.0;//TODO: actual value raising the arm speed
	public final static double ARM_SPEED_LOWER = 0.6; //TODO: actual value lowering the arm speed
	public static final double PID_PERIOD = 0.02; //This should be the same as the robot period, i.e. 20ms

	//Drive PID variables
	public final static double DRIVETRAIN_P = 0.04;
	public final static double DRIVETRAIN_I = 0.01;
	public final static double DRIVETRAIN_D = 0.01;

	//Gripper Wrist PID variables
	//TODO PID tune gripper
	public final static double WRIST_P = 0.1;
	public final static double WRIST_I = 0.0;
	public final static double WRIST_D = 0.0;

	//Gripper Wrist Setpoints
	//TODO find appropriate values: wrist
	public final static int FULLY_FOLDED_WRIST_SETPOINT = 0;
	public final static int EXPEL_WRIST_SETPOINT = 1100;
	public final static int INTAKE_WRIST_SETPOINT = 30;
	
	//Arm setpoints
	//TODO insert appropriate values: arm
	public final static int INITIAL_ARM_POSITION = 0;
	public final static int BOTTOM_HATCH_ARM_SETPOINT = 0;
	public final static int BOTTOM_PORT_ARM_SETPOINT = 0;
	public final static int MIDDLE_HATCH_ARM_SETPOINT = 0;
	public final static int MIDDLE_PORT_ARM_SETPOINT = 0;
	public final static int HIGH_HATCH_ARM_SETPOINT = 0;
	public final static int HIGH_PORT_ARM_SETPOINT = 0;
	public final static int CARGO_INTAKE_ARM_SETPOINT = 0;

	//Arm PID variables
	//TODO PID tune arm
	public final static double ARM_P = 0.0;
	public final static double ARM_I = 0.0;
	public final static double ARM_D = 0.0;
	public final static double ARM_F = 0.0;
	
	//Robot components

	//Drive train
	public static  SpeedController frontLeftDrive;
	public static  SpeedController frontLeftSteer;
	public static  Encoder frontLeftEncoder;
	public static  SwerveEnclosure frontLeft;
	
	public static  SpeedController frontRightDrive;
	public static  SpeedController frontRightSteer;
	public static  Encoder frontRightEncoder;
	public static  SwerveEnclosure frontRight;
	
	
	public static  SpeedController backRightDrive;
	public static  SpeedController backRightSteer;
	public static  Encoder backRightEncoder;
	public static  SwerveEnclosure backRight;

	public static  SpeedController backLeftDrive;
	public static  SpeedController backLeftSteer;
	public static  Encoder backLeftEncoder;
	public static  SwerveEnclosure backLeft;
	
	public static  SwerveDrive swerve;

	//gripper wrist 
	public static VictorSPX gripperWrist;
	public static Encoder wristEncoder;
	public static DigitalInput wristLimitSwitch;

	//gripper wheels
	public static VictorSPX gripperWheels;

	//arm
	public static TalonSRX arm;
	public static Encoder armEncoder;
	public static DigitalInput lowerArmLimitSwitch; 
	public static DigitalInput upperArmLimitSwitch;

	//Finger
	public static DoubleSolenoid finger;
	public static Compressor compressor;

	//sensors
	public static PigeonIMU gyro;
	public static UltrasonicSensor ultra;
	
	public static void init() {
		//drive train 
		frontLeftDrive = new Spark(0);
		frontLeftSteer = new Talon(1);
		frontLeftEncoder = new Encoder(0,1);
		frontLeft = new DriveEnclosure("front left", frontLeftDrive, frontLeftSteer, 415, frontLeftEncoder,0);
		
		frontRightDrive = new Spark(2);
		frontRightDrive.setInverted(true);
		frontRightSteer = new Talon(3);
		
		frontRightEncoder = new Encoder(2,3);
		frontRight = new DriveEnclosure("front right", frontRightDrive, frontRightSteer, 415, frontRightEncoder,0);
		
		
		backRightDrive = new Spark(4);
		backRightSteer = new Talon(5);
		backRightDrive.setInverted(true);
		backRightEncoder = new Encoder(4,5);
		backRight = new DriveEnclosure("back right", backRightDrive, backRightSteer, 415, backRightEncoder,0);

		backLeftDrive = new Spark(6);
		backLeftSteer = new Talon(7);
		backLeftEncoder = new Encoder(6,7);
		backLeft = new DriveEnclosure("back left", backLeftDrive, backLeftSteer, 178, backLeftEncoder,0);

		swerve = new SwerveDrive(frontLeft,frontRight,backRight,backLeft, WIDTH, LENGTH);
		
		//Gripper wrist
		gripperWrist = new VictorSPX(1);
		wristEncoder = new Encoder(8, 9);
		LiveWindow.add(wristEncoder);
		wristLimitSwitch = new DigitalInput(13); //port 3 on the MXP

		//Gripper wheels
		gripperWheels = new VictorSPX(2);

		arm = new TalonSRX(3);
		arm.setInverted(false);
		armEncoder = new Encoder(11,10); //port 0 and port 1 on the MXP
		lowerArmLimitSwitch = new DigitalInput(14);//port 4 MXP

		//Gripper finger(pneumatic)
		finger = new DoubleSolenoid(4, 0, 1);

		compressor = new Compressor(0);
        compressor.setClosedLoopControl(true);

		//sensors
		gyro = new PigeonIMU(0);

		SerialPort ultraSerial = new SerialPort(9600, Port.kOnboard, 8, Parity.kNone, StopBits.kOne);
		ultraSerial.reset();
		ultra = new UltrasonicSensor(ultraSerial);


	}
}
