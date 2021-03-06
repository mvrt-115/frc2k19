/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmState;

public class ManualControl extends Command {
  public ManualControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  protected void initialize() {
    Robot.arm.updateState(ArmState.MANUAL);
  }

  protected void execute() {}

  protected boolean isFinished() {
    return !Robot.oi.getManual();
  }

  protected void end() {
   // Robot.arm.updateState(ArmState.HOLD);
  //  Robot.arm.setArmSetpoint(Robot.arm.getArmEncoderValue());
  }

  protected void interrupted() {}
}
