/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3319.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team3319.robot.subsystems.Arm;
import org.usfirst.frc.team3319.robot.subsystems.Climber;
import org.usfirst.frc.team3319.robot.subsystems.DriveTrain;
import org.usfirst.frc.team3319.robot.subsystems.Finger;
import org.usfirst.frc.team3319.robot.subsystems.GripperWheels;
import org.usfirst.frc.team3319.robot.subsystems.GripperWrist;
import org.usfirst.frc.team3319.robot.subsystems.HatchHook;
import org.usfirst.frc.team3319.robot.vision.ResizingPipeline;
import org.usfirst.frc.team3319.swerve.math.CentricMode;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
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
	public static HatchHook hatchHook;
	public static Climber climber;

	//OI	
	public static OI oi;

	//Network tables
	private NetworkTableEntry lineEntryVectorX;
	private NetworkTableEntry lineEntryVectorY;
	private NetworkTableEntry lineEntryX;
	private NetworkTableEntry lineEntryY;
	
	//these values are a vector that consitutes a line and a point on that line
	private double vx;
	private double vy;
	private double x;
	private double y;
	private NetworkTable networkTable;
	private NetworkTableInstance networkTableInstance;



	//Vision processing
	
	public static UsbCamera camera;
	public static ResizingPipeline tapeRecognitionPipeline;
	public static VisionThread visionThread;
	public static VisionRunner<ResizingPipeline> visionRunner;
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
		arm = new Arm(RobotMap.ARM_P,RobotMap.ARM_I,RobotMap.ARM_D,RobotMap.ARM_F);
		finger = new Finger();
		hatchHook = new HatchHook(); 
		climber = new Climber();

		//create OI
		oi = new OI();


		SmartDashboard.putData(((Sendable) driveTrain));
		SmartDashboard.putData((Sendable) gripperWrist);
		SmartDashboard.putData((Sendable) arm);
		LiveWindow.add(gripperWrist);
		LiveWindow.add(arm);

		networkTableInstance = NetworkTableInstance.getDefault();
		System.out.println(networkTableInstance.isConnected());
		networkTable = networkTableInstance.getTable("LineData");
		lineEntryVectorX = networkTable.getEntry("vx");
		lineEntryVectorY = networkTable.getEntry("vy");
		lineEntryX = networkTable.getEntry("x");
		lineEntryY = networkTable.getEntry("y");
		
		networkTable.addEntryListener((table, key, entry, value, flags) -> {
			imageUpdated();
			}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
		

		//SmartDashboard.putBoolean("Enable compressor? ", true);

		//Vision processing logic
		streamSource = CameraServer.getInstance().startAutomaticCapture();
		
		
		outputToDashboard = CameraServer.getInstance().putVideo("Processed footage", 320, 240);
		outputToDashboard.setFPS(30);

		tapeRecognitionPipeline = new ResizingPipeline();

		visionRunner = new VisionRunner<ResizingPipeline>(streamSource, tapeRecognitionPipeline, pipeline ->
			{
				imageUpdated();	
			}
		);
		visionThread = new VisionThread(visionRunner);
		visionThread.start();
	}
	
	/** This function is called each time that the vision coprocessor updates 
	 *  the data related to line following.
	*/
	private void imageUpdated() {
		image = tapeRecognitionPipeline.resizeImageOutput();

		//SmartDashboard.putBoolean("Displaying line: ",driveTrain.getDisplayLine());

		if (driveTrain.getDisplayLine()) {
			vx =  lineEntryVectorX.getDouble(vx);
			vy =  lineEntryVectorY.getDouble(vy);
			x =  lineEntryX.getDouble(x);
			y =  lineEntryY.getDouble(y);


			int m = 1000;
			//lineImage = new Mat(240,320,CvType.CV_8UC1, new Scalar(255,255,255));

			Imgproc.line(image, 
					new Point(x-m*vx, y-m*vy), 
					new Point(x+m*vx, y+m*vy),
					new Scalar(0,0,255),
					2);
			
		}

		outputToDashboard.putFrame(image);

	}
	
	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		finger.retractFinger();
 	}

	@Override
	public void disabledPeriodic() {
		//This must be here for sensor data to be updated to the dashboard while disabled, do not remove it
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
		teleopInit();
		//These sensors should only be reset at the beginning of the match, not also at the beinning of teleop
		driveTrain.resetEncoders();
		driveTrain.resetGyro();
		gripperWrist.resetEncoder();
		arm.resetEncoder();
		//hatchHook.getHatch();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		//We are just using the camera to drive during the sandstorm, so no special action with an autonomous mode is necessary
		teleopPeriodic();

	}

	@Override
	public void teleopInit() {
		driveTrain.setCentricMode(SmartDashboard.getBoolean("FIELD Centric",false) ? CentricMode.FIELD : CentricMode.ROBOT);	
		driveTrain.resetGyro();
		gripperWrist.resetEncoder();
		arm.resetEncoder();
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
		arm.resetEncoder();
	}
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	@Override 
	public void robotPeriodic() { 
	}
}
