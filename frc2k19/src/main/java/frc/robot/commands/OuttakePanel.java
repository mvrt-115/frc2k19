/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Hardware;
import frc.robot.Robot;

public class OuttakePanel extends Command {
  
  private int state;
  public OuttakePanel() {
    state  = 1;

  }

  protected void initialize() {
    state = 1;
     Robot.panelIntake.outtakePanel(1);
    //Hardware.claw.set(Value.kForward);
  //  Robot.panelIntake.pancakeLeft.set(Value.kReverse);

    setTimeout(0.5);  //.4
  
  }

  protected void execute() {

   if(state ==1 && isTimedOut()){
      state = 2;
      Robot.panelIntake.outtakePanel(2);
      new FlashLimelight().start();
      setTimeout(1); // .25
     }
    else if(state ==2 && isTimedOut()){
       state = 3;
        Robot.panelIntake.outtakePanel(3);
       setTimeout(2.5);
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
