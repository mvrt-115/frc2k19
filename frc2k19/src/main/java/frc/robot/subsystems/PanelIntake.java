/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Hardware;

/**
 * Add your docs here.
 */

public class PanelIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public PanelIntake() {
    Hardware.claw = new DoubleSolenoid(1, 1, 0);
    Hardware.activeRelease = new DoubleSolenoid(1, 3, 2);

  }

  public void extendIntake() {
    Hardware.activeRelease.set(Value.kForward);
    Timer.delay(0.5);
    Hardware.claw.set(Value.kForward);
    Timer.delay(0.5);

    Hardware.activeRelease.set(Value.kReverse);
  }

  public void retractIntake() {
    Hardware.activeRelease.set(Value.kForward);
    Timer.delay(0.5);
    Hardware.claw.set(Value.kReverse);
    //Timer.delay(0.5);
    Hardware.activeRelease.set(Value.kReverse);
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
