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

  protected void initialize() {
    state = 1;
    Robot.panelIntake.outtakePanel(1);
    setTimeout(0.4);
  
  }

  protected void execute() {

    if(state ==1 && isTimedOut()){
      state = 2;
      Robot.panelIntake.outtakePanel(2);
      setTimeout(0.25);
    }
    else if(state ==2 && isTimedOut()){
       state = 3;
       Robot.panelIntake.outtakePanel(3);
       setTimeout(1.5);
    }
  }

  protected boolean isFinished() {
    if(state == 3 && isTimedOut())
      return true;
    return false;
  }

  protected void end() {
    Robot.panelIntake.outtakePanel(4);
  }

  protected void interrupted() {}
}
