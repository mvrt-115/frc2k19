/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveWithJoystick extends Command {
 
 
  public DriveWithJoystick() {
     requires(Robot.drivetrain);
  }

  protected void initialize() {}


  protected void execute() {
    if(Robot.oi.getVisionTurn()) {
      Robot.drivetrain.setCamMode(0);
      double angle = Robot.drivetrain.getAngle();
      Robot.drivetrain.driveWithTarget(Robot.oi.getThrottle(), angle);
    }
    else {
      Robot.drivetrain.setCamMode(1);
      Robot.drivetrain.cheesyDriveWithJoystick(Robot.oi.getThrottle(), Robot.oi.getWheel(), Robot.oi.getQuickTurn());   
    }
  }

  
  protected boolean isFinished() {
     return false;
  }

  protected void end() {
      Robot.drivetrain.setLeftRightMotorOutputs(0, 0);
  }
  
  protected void interrupted() {
      end();
  }
}