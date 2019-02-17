/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {

    public static final double kHoldVoltage = 1.5/12;
	public static final int kMidEncoderValue  = 11000; //Find Exact Value
	
	public static final int kBottomTolerance = 3000;
	public static final int kSetpointTolerance = 30;
	public static final int kBackTolerance = 21000;

	public static final int kTimeoutMs = 0;
	public static final int kSlotIdx = 0;
	public static final int kPIDLoopIdx = 0;
	
	public static final double kArmP = 0.25;
	public static final double kArmD = 0;
	public static final double kArmF = 0;

	public static final double kVisionTurnP = .02;

	public static final int kMaxCruiseVelocity = 15000;

	public static final int kZero = 2600;  
	public static final int kCargoShipFront = 4000;
	public static final int kCargoShipBack = 12500;
	public static final int kCargoRocketBack = 4800; 
	public static final int kCargoIntakeLevel = 21000;

	public static final int kDriveFrontLeft = 4;
	public static final int kDriveBackLeft = 8;
	public static final int kDriveFrontRight = 1;
	public static final int kDriveBackRight = 2;


	public static final int kArmOne = 34;
	public static final int kArmTwo = 35;
	public static final int kArmThree = 33;
	public static final int kArmFour = 36;

	public static final int kCargoIntakeTop = 37;
	public static final int kCargoIntakeBottom = 38;

	//Pathfinder Constants

	public static final double TIME_STEP = 0.05; //Seconds
	public static final double WHEEL_DIAMETER = .1524; //Meters 
	public static final double WHEELBASE_WIDTH = 0.559; //Meters 
	public static final double MAX_VELOCITY = 4.572; // m/s  8
	public static final double MAX_ACCEL = 1.99;  // 	8
	public static final double MAX_JERK = 60; // 60
	public static final double MAX_OFFSET = .20;
	public static final double GEAR_RATIO = 8.8;
	public static final int TICKS_PER_ROTATION = 42 * 9; 

	
//	public static final double CAMERA_HEIGHT = 0.0508; //Meters	// 2inches
//	public static final double TARGET_HEIGHT = 0.7239; //Meters	//30 inches  // 71.4cm to Hatch Height
//	public static final double CAMERA_ANGLE = 0;

	

}
