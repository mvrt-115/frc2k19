/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ShootCargo extends Command {
  public ShootCargo() {}

  protected void initialize() {}

  protected void execute() {
    Robot.cargoIntake.shootCargo();
  }

  protected boolean isFinished() {
    return !Robot.oi.getShootCargo();
  }

  protected void end() {
    Robot.cargoIntake.stop();
  }
  protected void interrupted() {}
}
