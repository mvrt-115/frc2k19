/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class GroundIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WPI_TalonSRX intake;
  public WPI_TalonSRX stowing;
  public DigitalInput breakbeam;
  public int dummyvalue;
  public int anotherdummyvalue;
  public int kTimeoutMs;
  public int kSlotIdx;
  public int kPIDLoopIdx;
  public int kStowingF;
  public int kStowingP;
  public int kStowingI;
  public int kStowingD;

  public static int stowedPosition;


  public GroundIntake()
  {
    intake = new WPI_TalonSRX(9);
    stowing = new WPI_TalonSRX(6);
    breakbeam = new DigitalInput(0);
    
    stowing.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, dummyvalue, anotherdummyvalue);
    stowing.setSensorPhase(true);
    stowing.setInverted(false);

    stowing.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
    stowing.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
        //no clue what these enums are 
        
    stowing.configNominalOutputForward(0, kTimeoutMs);
    stowing.configNominalOutputReverse(0, kTimeoutMs);
    stowing.configPeakOutputForward(1, kTimeoutMs);
    stowing.configPeakOutputReverse(-1, kTimeoutMs);
    
    stowing.selectProfileSlot(kSlotIdx, kPIDLoopIdx);

    stowing.config_kF(0, kStowingF, kTimeoutMs);
                               
    stowing.config_kP(0, kStowingP, kTimeoutMs);
    stowing.config_kI(0, kStowingI, kTimeoutMs);
    stowing.config_kD(0, kStowingD, kTimeoutMs);
        
    stowing.configMotionCruiseVelocity(15000, kTimeoutMs);
    stowing.configMotionAcceleration(6000, kTimeoutMs);
        
    stowing.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
  }

  public void stowIntake()
  {
    stowedPosition = stowing.getSelectedSensorPosition() + 1024;
    stowing.set(ControlMode.Position, stowedPosition);
  }

  public void returnStow()
  {
    stowedPosition = stowing.getSelectedSensorPosition() - 1024;
    stowing.set(ControlMode.Position, stowedPosition);
  }

  public void stopStowing()
  {
    stowing.stopMotor();
  }

  public void doIntake()
  {
    intake.set(1);
  }
  
  public void stopIntake()
  {
    intake.stopMotor();
  }

  /* public void drive()
  {
    intake.set(Robot.oi.getAnotherThrottle());
    stowing.set(Robot.oi.getThrottle());
  }*/

  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void stop()
  {
    intake.stopMotor();
    stowing.stopMotor();
  }
}