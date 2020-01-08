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
import frc.robot.Robot.RobotState;
import frc.robot.util.Limelight.LED_MODE;

public class FollowPath extends Command {
  
  private String path;
  private boolean isBackwards;
  
  public FollowPath() {
    requires(Robot.drivetrain);
    this.path = Robot.autonPath;
    this.isBackwards = false;
  }

  // Called just before this Command runs the first time
  @Override 
  protected void initialize() {
    path  = Robot.autonPath;
    if(Robot.currState == RobotState.TELEOP || Robot.autonPath == ""){
      end();
    }
    

    Hardware.backRightEncoder.setPosition(0);
    Hardware.backLeftEncoder.setPosition(0);
    Hardware.frontRightEncoder.setPosition(0);
    Hardware.frontLeftEncoder.setPosition(0);
     
    Robot.drivetrain.initializePathFollower(path);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

        
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
      //  return Hardware.leftFollower.isFinished() || Hardware.rightFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.notifier.stop();
    Robot.drivetrain.setLeftRightMotorOutputs(0, 0);
  /*  Hardware.backRightEncoder.setPosition(0);
    Hardware.backLeftEncoder.setPosition(0);
    Hardware.frontRightEncoder.setPosition(0);
    Hardware.frontLeftEncoder.setPosition(0);
   */ 
    new FlashLimelight().start();
    new DriveWithJoystick().start();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
