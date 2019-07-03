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

public class FlashLimelight extends Command {
  public FlashLimelight() {}

  protected void initialize() {
    setTimeout(.3);
  }

  protected void execute() {
    Robot.drivetrain.limelight.setLED(LED_MODE.BLINKING);
  }

  protected boolean isFinished() {
    if(isTimedOut())
      return true;
    return false;
  }

  protected void end() {
    Robot.drivetrain.limelight.setLED(LED_MODE.OFF);
  }

  protected void interrupted() {}
}
