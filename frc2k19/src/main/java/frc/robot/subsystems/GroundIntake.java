/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Hardware;

/**
 * Add your docs here.
 */
public class GroundIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  public GroundIntake() {
    Hardware.groundIntake = new TalonSRX(Constants.kGroundIntake); 
    Hardware.groundPivot = new TalonSRX(Constants.kGroundPivot); 

    Hardware.groundPivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    Hardware.groundPivot.setSensorPhase(false);

    Hardware.groundPivot.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    Hardware.groundPivot.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    Hardware.groundPivot.configNominalOutputForward(0, Constants.kTimeoutMs);
    Hardware.groundPivot.configNominalOutputReverse(0, Constants.kTimeoutMs);

    Hardware.groundPivot.configPeakOutputForward(0.4, Constants.kTimeoutMs);
    Hardware.groundPivot.configPeakOutputReverse(-0.4, Constants.kTimeoutMs);

    Hardware.groundPivot.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    Hardware.groundPivot.config_kF(0, Constants.kPivotF, Constants.kTimeoutMs);
    Hardware.groundPivot.config_kP(0, Constants.kPivotP, Constants.kTimeoutMs);
    Hardware.groundPivot.config_kD(0, Constants.kPivotD, Constants.kTimeoutMs);
    
    Hardware.groundPivot.configAllowableClosedloopError(Constants.kSlotIdx, 90, Constants.kTimeoutMs);

    Hardware.groundPivot.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    Hardware.groundPivot.configMotionAcceleration(6000, Constants.kTimeoutMs);

    Hardware.groundPivot.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  public void stowIntake() {
    Hardware.groundPivot.set(ControlMode.MotionMagic, Constants.kStowIntake);
  }

  public void retractIntake() {
    Hardware.groundPivot.set(ControlMode.MotionMagic, Constants.kDeployIntake);
  }

  public void extendIntake() {
    if(Math.abs(Constants.kExtendIntake - Hardware.groundPivot.getSelectedSensorPosition(0)) < Constants.kGroundTolerance)
      Hardware.groundIntake.set(ControlMode.PercentOutput, -0.07);
    else
      Hardware.groundPivot.set(ControlMode.MotionMagic, Constants.kExtendIntake);
  }

  public void intakeHatch(){
    Hardware.groundIntake.set(ControlMode.PercentOutput, 0.3);
  }

  public void outtakeHatch() {
    Hardware.groundIntake.set(ControlMode.PercentOutput, -0.3);
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
