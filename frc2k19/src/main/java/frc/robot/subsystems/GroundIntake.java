/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.commands.Pickup;

import frc.robot.commands.DriveWithJoystick;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class GroundIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WPI_TalonSRX intake;
  public WPI_TalonSRX stowing;
  public DigitalInput breakbeam;
  public AnalogOutput encoder;

  public GroundIntake()
  {
    intake = new WPI_TalonSRX(9);
    stowing = new WPI_TalonSRX(6);
    breakbeam = new DigitalInput(0);
    encoder = new AnalogOutput(1);
  }

  public void stowIntake()
  {
    stowing.set(0.5);
    Timer.delay(2);
    stowing.stopMotor();
  }

  public void doIntake()
  {
    intake.set(1);
  }

  public void stopIntake()
  {
    intake.stopMotor();
  }

  /* public void drive()
  {
    intake.set(Robot.oi.getAnotherThrottle());
    stowing.set(Robot.oi.getThrottle());
  }*/

  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Pickup());

  }

  public void stop()
  {
    intake.stopMotor();
    stowing.stopMotor();
  }
}