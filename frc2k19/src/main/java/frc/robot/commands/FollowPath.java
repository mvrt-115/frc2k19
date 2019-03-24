/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Hardware;
import frc.robot.Robot;

public class FollowPath extends Command {
  public FollowPath() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivetrain.initializePathFollower();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //if(Hardware.leftFollower.isFinished() || Hardware.rightFollower.isFinished()){
    //  double angle = Robot.drivetrain.getAngle();
    //  Robot.drivetrain.driveWithTarget(Robot.oi.getThrottle(), angle); // throttle doesn't matter, constant being used
    //}
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Hardware.leftFollower.isFinished() || Hardware.rightFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.setLeftRightMotorOutputs(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
