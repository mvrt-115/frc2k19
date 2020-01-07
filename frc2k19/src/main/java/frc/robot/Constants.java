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
	
	public static final int kInvertedMotors = -1; // comp bot =-1 practice = 1
	public static final boolean kCompbot = true;

	//General PID Constants
	public static final int kMaxCruiseVelocity = 15000;
	public static final int kTimeoutMs = 0;
	public static final int kSlotIdx = 0;
	public static final int kPIDLoopIdx = 0;
	
	//Arm PID Constants
	public static final double kArmP = 0.15;   //0.25
 	public static final double kArmD = 0.06;  //0.058
	public static final double kArmF = 0;
	public static final double kHoldVoltage = 1.5/12;

	// Climb PID Constants
	public static final double kClimbP = .7;
	public static final double kClimbF = 0.0;
	public static final double kClimbD = 0.0;

	//Arm Setpoints
	public static final int kZero = 2300;  
	public static final int kCargoShipFront = 4000; //3500
	public static final int kCargoShipBack = 12500;
	public static final int kCargoRocketBack = 5000; 
	public static final int kCargoIntakeLevel = 20500;
	public static final int kMidEncoderValue  = 11000; 
	public static final int kBottomTolerance = 2100;
	public static final int kBackTolerance = 21000;
 
	//Climb Constants
	public static final int kClimbZero = 0;
	public static final int kClimb = 3500; // 3500
	public static final int kClimbPush = 7000; // 6000

	// Motor Controller ID's
	public static final int kDriveFrontLeft = 5;  //4 5			2
	public static final int kDriveBackLeft = 2;   //8 7			1
	public static final int kDriveFrontRight = 6; //1 3			4
 	public static final int kDriveBackRight = 1; //2 6 			8

	public static final int kGroundPivot = 0; // 2   31 
	public static final int kGroundIntake = 0; // 20  32 

	public static final int kArmOne = 32;
	 //34 8
	public static final int kArmTwo = 31; //35 4
	public static final int kArmThree = 40; //33 22
	public static final int kArmFour = 30; //36 18 

	public static final int kLeftClimb = 33;	//	32 20
	public static final int kRightClimb = 34;	//	39 9

	public static final int kRightClimbRoller = 39; // 37 19
	public static final int kLeftClimbRoller = 35; // 31 2 

	public static final int kCargoIntakeTop = 39; //30 19
	public static final int kCargoIntakeBottom = 36; //38  21
	
	//Pathfinder Constants
	public static final double TIME_STEP = 0.02; //Seconds
	public static final double WHEEL_DIAMETER = .56; //Feet 
	public static final double WHEELBASE_WIDTH = 1.87; //Feet 
	public static final double MAX_VELOCITY = 17.5; // ft/s  8
	public static final double MAX_ACCEL = 6.7;  // 	
	public static final double MAX_JERK = 197; // 
	public static final double MAX_OFFSET = .20;
	public static final double GEAR_RATIO = 8.8;
	public static final int TICKS_PER_ROTATION = 42 * 9; 

	//Vision Targets
	public static final double kVisionThrottleP = 0;
	public static final double kVisionWheelP = 1.25/30;
	public static final double kMaxArea = 0;		

}
