/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleFunction;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;


import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.commands.DriveWithJoystick;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Drivetrain extends Subsystem {

  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  public AHRS navX;

  private double quickStopAccumulator = 0.0;
  private double wheelDeadBand = 0.03;
  private double throttleDeadBand = 0.02;

  public double xAngle;
  public double yAngle;

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

   Hardware.leftFollower.configurePIDVA(0,0,0, 1/Constants.MAX_VELOCITY, 0);
   Hardware.rightFollower.configurePIDVA(0,0,0, 1/Constants.MAX_VELOCITY, 0);

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    navX = new AHRS(SPI.Port.kMXP);
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
    double leftOutput = angle * Constants.kVisionTurnP;
    double rightOutput = -angle * Constants.kVisionTurnP;

    if(Robot.arm.isInverted)
      throttle *= -1;
 
    leftOutput += throttle;
    rightOutput += throttle;

    setLeftRightMotorOutputs(Constants.kInvertedMotors * leftOutput, Constants.kInvertedMotors * (-rightOutput));
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

  public void updateMotionFollowing() {
    double leftOutput =  Hardware.leftFollower.calculate((int) (-Robot.drivetrain.getleftEncoderPosition() * 42));
    double rightOutput = Hardware.rightFollower.calculate((int) (Robot.drivetrain.getRightEncoderPosition() * 42));

   /* double gyro = navX.getAngle();
    double desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(Hardware.leftFollower.getHeading()));
    double angleDifference = -(desiredHeading - gyro);
    double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

    SmartDashboard.putNumber("DESIRED HEADING", desiredHeading);
    SmartDashboard.putNumber("ANGLE ERROR", desiredHeading - gyro);

    leftOutput += turn;
    rightOutput -= turn;

    */
    setLeftRightMotorOutputs(leftOutput, -rightOutput);
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoystick());
  }

}