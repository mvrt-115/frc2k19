/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ExtendClimb extends Command {
  
  int stage;
  public ExtendClimb() {
    stage = 0;
   }

   protected void initialize() {
    Robot.climber.climb();
    System.out.print("Climbing started");
    stage = 0;
    setTimeout(1);
  }

  protected void execute() {
    if(isTimedOut()){
       Robot.climber.push();
    //   Robot.climber.driveWheels();
    }
  }

  protected boolean isFinished() {
    return !Robot.oi.getExtendClimb();
  }

  protected void end() {
    Robot.climber.stopWheels();
    Robot.climber.retract();
    System.out.print("Climbing ended\n\n");
  }

  protected void interrupted() {}
}
