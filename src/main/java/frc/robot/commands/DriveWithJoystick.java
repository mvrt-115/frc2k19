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

public class DriveWithJoystick extends Command {
 
 
  public DriveWithJoystick() {
     requires(Robot.drivetrain);
  }

  protected void initialize() {
      Robot.drivetrain.limelight.setLED(LED_MODE.OFF);  
      
      if(Robot.arm.isInverted)
        Robot.drivetrain.limelight.setPipeline(PIPELINE_STATE.BACK_DRIVER);
      else
       Robot.drivetrain.limelight.setPipeline(PIPELINE_STATE.FRONT_DRIVER);
  }


  protected void execute() {
     Robot.drivetrain.cheesyDriveWithJoystick(0.8 *Robot.oi.getThrottle(), 0.6 * Robot.oi.getWheel(), Robot.oi.getQuickTurn());   
   // Robot.drivetrain.setLeftRightMotorOutputs(Robot.oi.getThrottle(), Robot.oi.getWheel());
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