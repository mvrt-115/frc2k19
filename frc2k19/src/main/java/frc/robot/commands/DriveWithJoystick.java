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
    // Robot.drivetrain.curvatureDrive(-Robot.oi.getThrottle(), Robot.oi.getWheel(), Robot.oi.getQuickTurn());
    Robot.drivetrain.cheesyDriveWithJoystick(-Robot.oi.getThrottle(), Robot.oi.getWheel(), Robot.oi.getQuickTurn());
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