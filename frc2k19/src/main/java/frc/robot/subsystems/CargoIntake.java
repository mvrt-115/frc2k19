/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class CargoIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DigitalInput breakbeam;

  public CargoIntake() {
    Hardware.cargoIntakeTop = new TalonSRX(Constants.kCargoIntakeTop);
    Hardware.cargoIntakeBottom = new TalonSRX(Constants.kCargoIntakeBottom);

    Hardware.cargoIntakeTop.setNeutralMode(NeutralMode.Brake);
    Hardware.cargoIntakeBottom.setNeutralMode(NeutralMode.Brake);
    
    breakbeam = new DigitalInput(3);
  } 

  public void intakeCargo() {
    Hardware.cargoIntakeTop.set(ControlMode.PercentOutput, 0.9);
    Hardware.cargoIntakeBottom.set(ControlMode.PercentOutput, -Constants.kInvertedMotors * 0.9);
  }

  public void outtakeCargo() {

    if(Robot.arm.setpoint == Constants.kCargoRocketBack){
      Hardware.cargoIntakeTop.set(ControlMode.PercentOutput, -0.77);
      Hardware.cargoIntakeBottom.set(ControlMode.PercentOutput, -Constants.kInvertedMotors *-0.77);
    }else if(Robot.arm.setpoint == Constants.kCargoShipFront){
      Hardware.cargoIntakeTop.set(ControlMode.PercentOutput, -0.5); // -0.6 for climbers
      Hardware.cargoIntakeBottom.set(ControlMode.PercentOutput, -Constants.kInvertedMotors * -0.5); // 0.6 for climbers
    } else if(Robot.arm.setpoint == Constants.kCargoShipBack){
       Hardware.cargoIntakeTop.set(ControlMode.PercentOutput, -0.4);
        Hardware.cargoIntakeBottom.set(ControlMode.PercentOutput, -Constants.kInvertedMotors * -0.4); 
    }else {
     // Hardware.cargoIntakeTop.set(ControlMode.PercentOutput, -0.3);
      Hardware.cargoIntakeBottom.set(ControlMode.PercentOutput, -Constants.kInvertedMotors * -0.3);
    }
  }

  public void shootCargo() {
    if(Robot.arm.setpoint == Constants.kCargoRocketBack){
      Hardware.cargoIntakeTop.set(ControlMode.PercentOutput, -0.9);
      Hardware.cargoIntakeBottom.set(ControlMode.PercentOutput, -Constants.kInvertedMotors *-0.9);
    }else {
      Hardware.cargoIntakeTop.set(ControlMode.PercentOutput, -0.3);
      Hardware.cargoIntakeBottom.set(ControlMode.PercentOutput, -Constants.kInvertedMotors *-0.4);
      
    }
  }

  //Test 
  public void stop() {
    Hardware.cargoIntakeTop.
    set(ControlMode.PercentOutput, 0);
    Hardware.cargoIntakeBottom.set(ControlMode.PercentOutput, 0);
  }

  public void log(){
    SmartDashboard.putBoolean("Cargo BreakBeam", getBreakBeam());

  }

  @Override
  public void initDefaultCommand() {
  }

  public boolean getBreakBeam(){
    return breakbeam.get();
  }
}
