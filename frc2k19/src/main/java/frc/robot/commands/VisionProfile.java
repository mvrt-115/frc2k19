/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import jaci.pathfinder.Waypoint;

public class VisionProfile extends Command {
  public VisionProfile() {

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double yDistance = Robot.drivetrain.getYDistance();
    double xDistance = Robot.drivetrain.getXDistance();
    double targetAngle = Robot.drivetrain.getFinalAngle() + Robot.drivetrain.getYaw();
    
    Waypoint[] points = new Waypoint[] { 
        new Waypoint(0, 0, Robot.drivetrain.getYaw()),
        new Waypoint(yDistance, xDistance, targetAngle)};
    
    new FollowProfile(points);
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
