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

public class IntakePanel extends Command {
  
  public IntakePanel() {}

  protected void initialize() {
    Robot.panelIntake.intakePanel();
    // Hardware.claw.set(Value.kReverse);
   // Robot.panelIntake.pancakeLeft.set(Value.kForward);
  }

  protected void execute() {}

  protected boolean isFinished() {
    return true;
  }

  protected void end() {}

  protected void interrupted() {}
}
