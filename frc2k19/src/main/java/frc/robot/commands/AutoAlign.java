/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.util.Limelight.LED_MODE;
import frc.robot.util.Limelight.PIPELINE_STATE;

public class AutoAlign extends Command {
  public AutoAlign() {
    requires(Robot.drivetrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() { 
    if(Robot.arm.isInverted)
      Robot.drivetrain.limelight.setPipeline(PIPELINE_STATE.BACK_VISION);
    else
      Robot.drivetrain.limelight.setPipeline(PIPELINE_STATE.FRONT_VISION);

    Robot.drivetrain.limelight.setLED(LED_MODE.ON);
    

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double angle = Robot.drivetrain.limelight.getAngle();
    if(!Robot.arm.isInverted){
      Robot.drivetrain.driveWithTarget(Robot.oi.getThrottle(), angle);
    }else {
      Robot.drivetrain.driveWithTarget(-Robot.oi.getThrottle(), angle);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.oi.getVisionTurn();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    new DriveWithJoystick().start();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
