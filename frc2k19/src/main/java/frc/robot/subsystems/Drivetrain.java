/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.DoubleFunction;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.commands.DriveWithJoystick;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class Drivetrain extends Subsystem {

  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  private int k_ticks_per_rev = 42 * 9;
  private double k_wheel_diameter = 0.56;
  private double k_max_velocity = 17.5;
  public String k_path_name = "";

  public AHRS navX;
  private Notifier notifier;

  private double quickStopAccumulator = 0.0;
  private double wheelDeadBand = 0.03;
  private double throttleDeadBand = 0.02;

  public double xAngle;
  public double yAngle;

  private SpeedControllerGroup leftGroup, rightGroup;

  private static final double SENSITIVITY = 0.90;
  private DoubleFunction<Double> limiter = limiter(-0.9, 0.9);

  public Drivetrain() {
    Hardware.frontLeft = new CANSparkMax(Constants.kDriveFrontLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
    Hardware.backLeft = new CANSparkMax(Constants.kDriveBackLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
    Hardware.frontRight = new CANSparkMax(Constants.kDriveFrontRight, CANSparkMaxLowLevel.MotorType.kBrushless);
    Hardware.backRight = new CANSparkMax(Constants.kDriveBackRight, CANSparkMaxLowLevel.MotorType.kBrushless);

    Hardware.frontLeftEncoder = new CANEncoder(Hardware.frontLeft);
    Hardware.frontRightEncoder = new CANEncoder(Hardware.frontRight);
    Hardware.backLeftEncoder = new CANEncoder(Hardware.backLeft);
    Hardware.backRightEncoder = new CANEncoder(Hardware.backRight);
    Hardware.leftFollower = new EncoderFollower();
    Hardware.rightFollower = new EncoderFollower();

    Hardware.frontLeftEncoder.setPosition(0);
    Hardware.frontRightEncoder.setPosition(0);
    Hardware.backLeftEncoder.setPosition(0);
    Hardware.backRightEncoder.setPosition(0);

    leftGroup = new SpeedControllerGroup(Hardware.frontLeft,Hardware.backLeft);
    rightGroup = new SpeedControllerGroup(Hardware.frontRight, Hardware.backRight);

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    navX = new AHRS(SPI.Port.kMXP);
    navX.zeroYaw();

    Hardware.frontLeft.setIdleMode(IdleMode.kCoast);
		Hardware.backLeft.setIdleMode(IdleMode.kCoast);
		Hardware.frontRight.setIdleMode(IdleMode.kCoast);
    Hardware.backRight.setIdleMode(IdleMode.kCoast);

    

  }

  public void initializePathFollower() {

    navX.zeroYaw();

    Hardware.frontLeft.setIdleMode(IdleMode.kBrake);
		Hardware.backLeft.setIdleMode(IdleMode.kBrake);
		Hardware.frontRight.setIdleMode(IdleMode.kBrake);
    Hardware.backRight.setIdleMode(IdleMode.kBrake);

    Trajectory left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");
    Trajectory right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left"); // temporary because WPI API is broken
    
    Hardware.leftFollower = new EncoderFollower(left_trajectory);
    Hardware.rightFollower = new EncoderFollower(right_trajectory);
  
    Hardware.leftFollower.configureEncoder((int)(Hardware.frontLeftEncoder.getPosition() * 42), k_ticks_per_rev, k_wheel_diameter);
    Hardware.leftFollower.configurePIDVA(0.29, 0.0, 0.0, 1 / k_max_velocity, 0);

    Hardware.rightFollower.configureEncoder((int)(-Hardware.frontLeftEncoder.getPosition() * 42), k_ticks_per_rev, k_wheel_diameter);
    Hardware. rightFollower.configurePIDVA(0.29, 0.0, 0.0, 1 / k_max_velocity, 0);
      
    notifier = new Notifier(this::followPath);
    notifier.startPeriodic(left_trajectory.get(0).dt);

  }

  public void followPath() {
    if (Hardware.leftFollower.isFinished() || Hardware.rightFollower.isFinished()) {
      SmartDashboard.putBoolean("stop", true);
      notifier.stop();
      leftGroup.set(0);
      rightGroup.set(0);

    } 
    
    else {
      SmartDashboard.putBoolean("start", true);

      double left_speed = Hardware.leftFollower.calculate((int)(Hardware.frontLeftEncoder.getPosition() * 42));
      double right_speed = Hardware.rightFollower.calculate((int)(-Hardware.frontRightEncoder.getPosition() * 42));
      double heading = navX.getAngle();
      double desired_heading = -Pathfinder.r2d(Hardware.leftFollower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading + heading);
      double turn =  0.8 * (-1.0/78.0) * heading_difference;
      leftGroup.set(left_speed + turn);
      rightGroup.set(-(right_speed - turn));
    }

  }

  public double getleftEncoderPosition() {
    return (Hardware.frontLeftEncoder.getPosition() + Hardware.backLeftEncoder.getPosition()) / 2;
  }

  public double getRightEncoderPosition() {
    return (Hardware.frontRightEncoder.getPosition() + Hardware.backRightEncoder.getPosition()) / 2;

  }

  public double getAngle() {
    return tx.getDouble(0);
  }

  public double getYaw() {
    return navX.getYaw();
  }

  public double getYDistance() {
    Number[] empty = { 0, 0, 0, 0, 0, 0 };
    Number newArray[] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran")
        .getNumberArray(empty);

    return (double) newArray[2];
  }

  public double getXDistance() {
    Number[] empty = { 0, 0, 0, 0, 0, 0 };
    Number newArray[] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran")
        .getNumberArray(empty);

    return (double) newArray[1];
  }

  public double getFinalAngle(){
    Number[] empty = { 0, 0, 0, 0, 0, 0 };
    Number newArray[] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getNumberArray(empty);
    return (double) newArray[5];
  }

  public void switchPipeline(int number) {

    if(number == 3 || number == 4)
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    else
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(number);
  }


  public void setLEDMode(int number) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(number);

  }

  public void curvatureDrive(double speed, double wheel, boolean quickTurn) {
    //drive.curvatureDrive(speed, wheel, quickTurn);
  }

  public void setLeftRightMotorOutputs(double left, double right) {
    Hardware.frontLeft.set(left);
    Hardware.backLeft.set(left);
    Hardware.frontRight.set(right);
    Hardware.backRight.set(right);
  }

  public void cheesyDriveWithJoystick(double throttle, double wheel, boolean quickturn) {
    wheel = handleDeadband(wheel, wheelDeadBand);
    throttle = handleDeadband(throttle, throttleDeadBand);


    double overPower;
    double angularPower;

    wheel = dampen(wheel, 0.5);
    wheel = dampen(wheel, 0.5);
    wheel = dampen(wheel, 0.5);

    if (quickturn) {
      if (Math.abs(throttle) < 0.2) {
        double alpha = 0.1;
        quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * limiter.apply(wheel) * 2;
      }
      overPower = 1.0;
      angularPower = wheel;
      angularPower *= 0.5;
    } else {
      overPower = 0.0;
      angularPower = Math.abs(throttle) * wheel * SENSITIVITY - quickStopAccumulator;
      angularPower *= 0.8;
      if (quickStopAccumulator > 1) {
        quickStopAccumulator -= 1;
      } else if (quickStopAccumulator < -1) {
        quickStopAccumulator += 1;
      } else {
        quickStopAccumulator = 0.0;
      }
    }

    double rightPwm = throttle-  Constants.kInvertedMotors * angularPower;
    double leftPwm = throttle +Constants.kInvertedMotors *angularPower;
    if (leftPwm > 1.0) {
      rightPwm -= overPower * (leftPwm - 1.0);
      leftPwm = 1.0;
    } else if (rightPwm > 1.0) {
      leftPwm -= overPower * (rightPwm - 1.0);
      rightPwm = 1.0;
    } else if (leftPwm < -1.0) {
      rightPwm += overPower * (-1.0 - leftPwm);
      leftPwm = -1.0;
    } else if (rightPwm < -1.0) {
      leftPwm += overPower * (-1.0 - rightPwm);
      rightPwm = -1.0;
    }
    setLeftRightMotorOutputs(Constants.kInvertedMotors * leftPwm, Constants.kInvertedMotors * (-rightPwm));
  }

  public void driveWithTarget(double throttle, double angle) {
    double yawSpeed = 1.4 * angle/ 30;

    cheesyDriveWithJoystick(-0.1, yawSpeed, false);
  }

  public double handleDeadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  private static double dampen(double wheel, double wheelNonLinearity) {
    double factor = Math.PI * wheelNonLinearity;
    return Math.sin(factor * wheel) / Math.sin(factor);
  }

  private DoubleFunction<Double> limiter(double minimum, double maximum) {
    if (maximum < minimum) {
      throw new IllegalArgumentException("The minimum value cannot exceed the maximum value");
    }
    return (double value) -> {
      if (value > maximum) {
        return maximum;
      }
      if (value < minimum) {
        return minimum;
      }
      return value;
    };
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoystick());
  }

}