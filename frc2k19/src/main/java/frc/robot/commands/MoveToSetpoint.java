/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class MoveToSetpoint extends InstantCommand {
  
  int setpointTickValue;
  
  public MoveToSetpoint(int setpointTickValue) {
    this.setpointTickValue = setpointTickValue;
  }

  protected void initialize() {
    if(setpointTickValue == Constants.kZero){
      Robot.arm.zeroArm();
    }
    else {
      Robot.arm.setArmSetpoint(setpointTickValue);  
    }
  }

  protected void execute() {}

  protected void end() {}

  protected void interrupted() {
    end();
  }
}
