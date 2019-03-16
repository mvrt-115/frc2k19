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
	
	public static final int kBottomTolerance = 2100;
	public static final int kSetpointTolerance = 30;
	public static final int kBackTolerance2 = 18000; 
	public static final int kBackTolerance = 21000;

	public static final int kTimeoutMs = 0;
	public static final int kSlotIdx = 0;
	public static final int kPIDLoopIdx = 0;
	
	public static final double kArmP = 0.15;   //0.25
 	public static final double kArmD = 0.06;  //0.058
	public static final double kArmF = 0;

	public static final double kPivotP = 0.475;
	public static final double kPivotD = 0.0;
	public static final double kPivotF = 0.0;
	public static final double kGroundTolerance = 1550;
	public static final double kStowTolerance = 50;

	public static final double kVisionTurnP = .02;

	public static final int kMaxCruiseVelocity = 15000;

	public static final int kZero = 2300;  

	public static final int kCargoShipFront = 4000;
	public static final int kCargoShipBack = 12500;
	public static final int kCargoRocketBack = 5000; 
	public static final int kCargoIntakeLevel = 20500;

	public static final int kStowIntake = 0;
	public static final int kDeployIntake = 500;
	public static final int kExtendIntake = 1550; 

	public static final int kDriveFrontLeft = 5;  //4 5
	public static final int kDriveBackLeft = 7;   //8 7
	public static final int kDriveFrontRight = 3; //1 3
 	public static final int kDriveBackRight = 6; //2 6 

	public static final int kGroundPivot = 2; // 2   31 
	public static final int kGroundIntake = 20; // 20  32 

	public static final int kArmOne = 8; //34 8
	public static final int kArmTwo = 4; //35 4
	public static final int kArmThree = 22; //33 22
	public static final int kArmFour = 18; //36 18 

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

	public static final int kCargoIntakeTop = 42; //30 42
	public static final int kCargoIntakeBottom = 21; //38  21

	public static final int kInvertedMotors = -1; // comp bot inverted
	public static final boolean kCompbot = true;

}
