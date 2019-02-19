/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Hardware;

/**
 * Add your docs here.
 */
public class CargoIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public CargoIntake() {
    Hardware.cargoIntakeTop = new TalonSRX(Constants.kCargoIntakeTop);
    Hardware.cargoIntakeBottom = new TalonSRX(Constants.kCargoIntakeBottom);

  }

  public void intakeCargo() {
    Hardware.cargoIntakeTop.set(ControlMode.PercentOutput, 0.6);
    Hardware.cargoIntakeBottom.set(ControlMode.PercentOutput, 0.6);
  }

  public void outtakeCargo() {
    Hardware.cargoIntakeTop.set(ControlMode.PercentOutput, -0.4);
    Hardware.cargoIntakeBottom.set(ControlMode.PercentOutput, -0.4);  
  }

  //Test 
  public void stop() {
    Hardware.cargoIntakeTop.set(ControlMode.PercentOutput, 0);
    Hardware.cargoIntakeBottom.set(ControlMode.PercentOutput, 0);  
  }

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
