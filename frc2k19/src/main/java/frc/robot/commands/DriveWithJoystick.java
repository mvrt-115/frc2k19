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
      if(Robot.arm.isInverted)
        Robot.drivetrain.switchPipeline(1);
      else
        Robot.drivetrain.switchPipeline(0);
      double angle = Robot.drivetrain.getAngle();
      Robot.drivetrain.driveWithTarget(Robot.oi.getThrottle(), angle);
    }
    else {
      if(Robot.arm.isInverted)
        Robot.drivetrain.switchPipeline(4);
      else
        Robot.drivetrain.switchPipeline(3);
      Robot.drivetrain.cheesyDriveWithJoystick(0.8 *Robot.oi.getThrottle(), 0.6 * Robot.oi.getWheel(), Robot.oi.getQuickTurn());   
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