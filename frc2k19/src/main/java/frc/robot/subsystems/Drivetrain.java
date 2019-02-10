/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleFunction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.commands.DriveWithJoystick;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  
  //CANSparkMax frontLeft, frontRight, backLeft, backRight;
  SpeedControllerGroup leftDrive;
  SpeedControllerGroup rightDrive;
  DifferentialDrive drive;

  private double quickStopAccumulator = 0.0;
  private double wheelDeadBand = 0.03;
  private double throttleDeadBand = 0.02;

  private static final double SENSITIVITY = 0.90;
  private DoubleFunction<Double> limiter = limiter(-0.9, 0.9);


  public Drivetrain() {
    Hardware.frontLeft = new CANSparkMax(Constants.kDriveFrontLeft,CANSparkMaxLowLevel.MotorType.kBrushless);
    Hardware.backLeft = new CANSparkMax(Constants.kDriveBackLeft,CANSparkMaxLowLevel.MotorType.kBrushless);
    Hardware.frontRight = new CANSparkMax(Constants.kDriveFrontRight,CANSparkMaxLowLevel.MotorType.kBrushless);
    Hardware.backRight = new CANSparkMax(Constants.kDriveBackRight,CANSparkMaxLowLevel.MotorType.kBrushless);

    leftDrive = new SpeedControllerGroup(Hardware.frontLeft, Hardware.backLeft);
    rightDrive = new SpeedControllerGroup(Hardware.frontRight, Hardware.backRight);
    drive = new DifferentialDrive(leftDrive, rightDrive);
  }

  public void curvatureDrive(double speed, double wheel, boolean quickTurn) {
    drive.curvatureDrive(speed, wheel, quickTurn);
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
      } else {
          overPower = 0.0;
          angularPower = Math.abs(throttle) * wheel * SENSITIVITY - quickStopAccumulator;
          if (quickStopAccumulator > 1) {
            quickStopAccumulator -= 1;
          } else if (quickStopAccumulator < -1) {
            quickStopAccumulator += 1;
          } else {
           quickStopAccumulator = 0.0;
        }
      }
  
      double rightPwm = throttle - angularPower;
      double leftPwm = throttle + angularPower;
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
      setLeftRightMotorOutputs(leftPwm, -rightPwm);
    }  

  public double handleDeadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  private static double dampen (double wheel, double wheelNonLinearity) {
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