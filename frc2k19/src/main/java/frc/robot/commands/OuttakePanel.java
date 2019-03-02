/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class OuttakePanel extends Command {
  
  private int state;
  public OuttakePanel() {
    state  = 1;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    state = 1;
    Robot.panelIntake.retractIntake(1);
    setTimeout(0.4);
  
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(state ==1 && isTimedOut()){
      state = 2;
      Robot.panelIntake.retractIntake(2);
      setTimeout(0.25);
    }
    else if(state ==2 && isTimedOut()){
       state = 3;
       Robot.panelIntake.retractIntake(3);
       setTimeout(3);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(state == 3 && isTimedOut())
      return true;
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.panelIntake.retractIntake(4);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
