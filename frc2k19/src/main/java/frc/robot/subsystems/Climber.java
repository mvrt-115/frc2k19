/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public enum ClimberState {
    ZEROED, RETRACTING, EXTENDING, PUSHING, CARGOSHOT
  };

  public ClimberState currState;

  public Climber() {
    Hardware.leftClimb = new TalonSRX(Constants.kLeftClimb);
    Hardware.rightClimb = new TalonSRX(Constants.kRightClimb);

    Hardware.leftClimb.setNeutralMode(NeutralMode.Brake);
    Hardware.rightClimb.setNeutralMode(NeutralMode.Brake);

    Hardware.rightClimbRoller = new TalonSRX(Constants.kLeftClimbRoller);
    Hardware.leftClimbRoller = new TalonSRX(Constants.kRightClimbRoller);
    
   // Hardware.leftClimb.follow(Hardware.rightClimb);
    Hardware.rightClimb.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    Hardware.rightClimb.setSensorPhase(true);
    Hardware.rightClimb.setInverted(true);
    Hardware.leftClimb.setInverted(false);

    Hardware.rightClimb.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    Hardware.rightClimb.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    Hardware.rightClimb.configNominalOutputForward(0, Constants.kTimeoutMs);
    Hardware.rightClimb.configNominalOutputReverse(0, Constants.kTimeoutMs);

    Hardware.rightClimb.configPeakOutputForward(0.9, Constants.kTimeoutMs);
    Hardware.rightClimb.configPeakOutputReverse(-0.9, Constants.kTimeoutMs);

    Hardware.rightClimb.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    Hardware.rightClimb.config_kF(0, Constants.kClimbF, Constants.kTimeoutMs);
    Hardware.rightClimb.config_kP(0, Constants.kClimbP, Constants.kTimeoutMs);
    Hardware.rightClimb.config_kD(0, Constants.kClimbD, Constants.kTimeoutMs);
    
    Hardware.rightClimb.configAllowableClosedloopError(Constants.kSlotIdx, 90, Constants.kTimeoutMs);

    Hardware.rightClimb.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    Hardware.rightClimb.configMotionAcceleration(6000, Constants.kTimeoutMs);

    Hardware.rightClimb.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    currState = ClimberState.RETRACTING;

    new Notifier(new Runnable() {

      public void run() {
        loop();
      }

    }).startPeriodic(0.005);

  }


  public void loop() {

    switch(currState){
      case ZEROED:
        break;
      case EXTENDING:
        Hardware.rightClimb.set(ControlMode.MotionMagic, Constants.kClimb);
        //Hardware.leftClimb.follow(Hardware.rightClimb);
        Hardware.leftClimb.set(ControlMode.MotionMagic, Constants.kClimb);
        driveWheels();
        break;
      case RETRACTING:
        stopWheels();
        //if(Hardware.rightClimb.getSelectedSensorPosition() < 2000){
         // currState = ClimberState.ZEROED;
        //}else {
         Hardware.rightClimb.set(ControlMode.MotionMagic, Constants.kClimbZero);
         Hardware.leftClimb.set(ControlMode.MotionMagic, Constants.kClimbZero);

        //}
        break; 
      case PUSHING:
          Hardware.rightClimb.set(ControlMode.MotionMagic, Constants.kClimbPush);
         // Hardware.leftClimb.follow(Hardware.rightClimb);
          Hardware.leftClimb.set(ControlMode.MotionMagic, Constants.kClimbPush);

          break;
      case CARGOSHOT:
      //    Hardware.rightClimb.set(ControlMode.MotionMagic, -500);
       //   Hardware.leftClimb.set(ControlMode.MotionMagic, -500);

          break;
    }


  }


  public void climb(){
    currState = ClimberState.EXTENDING;
  }

  public void push() {
    currState = ClimberState.PUSHING;
  }

  public void retract() {
    currState = ClimberState.RETRACTING;
  }

  public void cargoShot(){
    currState = ClimberState.CARGOSHOT;
  }

  public void stopWheels(){
    Hardware.rightClimbRoller.set(ControlMode.PercentOutput, 0);
    Hardware.leftClimbRoller.set(ControlMode.PercentOutput, 0);
  }

  public void driveWheels(){
    Hardware.rightClimbRoller.set(ControlMode.PercentOutput, -1);
    Hardware.leftClimbRoller.set(ControlMode.PercentOutput, -1);
  }

  public void log(){
    SmartDashboard.putNumber("Climber Encoder Value", Hardware.rightClimb.getSelectedSensorPosition(0));

   SmartDashboard.putNumber("Left Climber Voltage", Hardware.leftClimb.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right Climber Voltage", Hardware.rightClimb.getMotorOutputVoltage());

    SmartDashboard.putNumber("Right Climber Output", Hardware.rightClimb.getMotorOutputPercent());
    SmartDashboard.putNumber("Left Climber Output", Hardware.leftClimb.getMotorOutputPercent());
   
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
