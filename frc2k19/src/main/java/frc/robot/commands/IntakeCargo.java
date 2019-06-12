/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeCargo extends Command {

  public enum CargoState {
    MANUAL, AUTO
  };

  public CargoState currState;  

  public IntakeCargo() {
    currState = CargoState.AUTO;
  }

  protected void initialize() {
    currState = CargoState.AUTO;

    if(Robot.oi.getIntakeCargo())
      currState = CargoState.MANUAL;
    else {
      setTimeout(1.2);
    }  
  }

  protected void execute() {

    if(Robot.oi.getIntakeCargo()){
      currState = CargoState.MANUAL;
      Robot.cargoIntake.intakeCargo();
    }else {
      if(isTimedOut())
        Robot.cargoIntake.intakeCargo();
    }
  }

  protected boolean isFinished() {
    if(currState == CargoState.MANUAL)
      return !Robot.oi.getIntakeCargo();
    else if(currState == CargoState.AUTO)
      return !Robot.cargoIntake.getBreakBeam();

    return false;
  }

  protected void end() {
    Robot.cargoIntake.stop();
  }

  protected void interrupted() {}
}
