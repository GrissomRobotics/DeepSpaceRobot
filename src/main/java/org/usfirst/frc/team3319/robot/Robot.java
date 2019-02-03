/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team3319.robot.custom.GripperSetpoint;
import org.usfirst.frc.team3319.robot.subsystems.Arm;
import org.usfirst.frc.team3319.robot.subsystems.DriveTrain;
import org.usfirst.frc.team3319.robot.subsystems.Finger;
import org.usfirst.frc.team3319.robot.subsystems.GripperWheels;
import org.usfirst.frc.team3319.robot.subsystems.GripperWrist;
import org.usfirst.frc.team3319.robot.vision.TapeRecognitionPipeline;
import org.usfirst.frc.team3319.swerve.math.CentricMode;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	//subsystems
	public static DriveTrain driveTrain;
	public static GripperWrist gripperWrist;
	public static GripperWheels gripperWheels;
	public static Arm arm;
	public static Finger finger;

	//OI	

	public static OI oi;

	//Vision processing
	public static UsbCamera camera;
	public static TapeRecognitionPipeline tapeRecognitionPipeline;
	public static VisionThread visionThread;
	public static VisionRunner<TapeRecognitionPipeline> visionRunner;
	private Mat image = new Mat(); //this must be initialized or a null pointer exception occurs below
	private CvSource outputToDashboard;
	private UsbCamera streamSource;


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//initialize the robot map
		RobotMap.init();

		//create subsystems
		driveTrain = new DriveTrain();
		gripperWrist = new GripperWrist(RobotMap.WRIST_P,RobotMap.WRIST_I,RobotMap.WRIST_D);
		gripperWheels = new GripperWheels();
		arm = new Arm(RobotMap.ARM_P,RobotMap.ARM_I,RobotMap.ARM_D);
		finger = new Finger();

		//create OI
		oi = new OI();


		SmartDashboard.putData(((Sendable) driveTrain));
		SmartDashboard.putData((Sendable) gripperWrist);
		SmartDashboard.putData((Sendable) gripperWheels);
		SmartDashboard.putData((Sendable) arm);
		LiveWindow.add(gripperWrist);
		LiveWindow.add(arm); 
		SmartDashboard.putData(Scheduler.getInstance());

		//Vision processing logic
		streamSource = CameraServer.getInstance().startAutomaticCapture();

		outputToDashboard = CameraServer.getInstance().putVideo("Processed footage", 160, 120);
		outputToDashboard.setFPS(25);

		tapeRecognitionPipeline = new TapeRecognitionPipeline();

		visionRunner = new VisionRunner<TapeRecognitionPipeline>(streamSource, tapeRecognitionPipeline, pipeline ->
			{
				pipelineFinished();	
			}
		);
		visionThread = new VisionThread(visionRunner);
		visionThread.start();
	}

	//This function is called each time the tape recognizer finishes processing a frame
	private void pipelineFinished() {
		image = tapeRecognitionPipeline.resizeImageOutput();
		Imgproc.drawContours(image, tapeRecognitionPipeline.filterContoursOutput(), 0 , new Scalar(0,0,0));//black outline
		outputToDashboard.putFrame(image);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		finger.retract();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		driveTrain.setCentricMode(SmartDashboard.getBoolean("Use gyro",true) ? CentricMode.FIELD : CentricMode.ROBOT);
		driveTrain.resetEncoders();
		driveTrain.disableTurnPID();
		driveTrain.resetGyro();
		finger.retract();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}
	
	@Override
	public void testInit() {
		gripperWrist.resetEncoder();
	}
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	
}