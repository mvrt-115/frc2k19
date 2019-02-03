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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  
  public enum ArmState {
		ZEROED,
    ZEROING,
		SETPOINT, 
		HOLD,
		MANUAL
  }; 

  public double setpoint;
  public DigitalInput limitSwitch;

  public ArmState currState = ArmState.ZEROED;

  public Arm() {

    setpoint = 0;

    Hardware.armOne = new TalonSRX(0);
    Hardware.armTwo = new TalonSRX(1);
    Hardware.armThree = new TalonSRX(2);
    Hardware.armFour = new TalonSRX(3);

    Hardware.armTwo.follow(Hardware.armOne);
    Hardware.armThree.follow(Hardware.armOne);
    Hardware.armFour.follow(Hardware.armOne);

    /* First choose the sensor. */
		Hardware.armOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		Hardware.armOne.setSensorPhase(false);

		/* Set relevant frame periods to be at least as fast as periodic rate. */
		Hardware.armOne.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		Hardware.armOne.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    
    Hardware.armOne.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		//		Hardware.ArmLeft.config_kF(0, Constants.kArmF, Constants.kTimeoutMs);
		Hardware.armOne.config_kP(0, Constants.kArmP, Constants.kTimeoutMs);
		Hardware.armOne.config_kD(0, Constants.kArmD, Constants.kTimeoutMs);

		//		Hardware.ArmLeft.config_kF(0, Constants.kArmF, Constants.kTimeoutMs);
		Hardware.armOne.config_kP(1, Constants.kArmP, Constants.kTimeoutMs);
		Hardware.armOne.config_kD(1, Constants.kArmD, Constants.kTimeoutMs);

		//Hardware.armOne.configAllowableClosedloopError(Constants.kSlotIdx, (int)(UnitConverter.convertInchesToTicks(1.1)), Constants.kTimeoutMs);

		/* set acceleration and vcruise velocity - see documentation */
		Hardware.armOne.configMotionCruiseVelocity(Constants.kMaxCruiseVelocity, Constants.kTimeoutMs);
		Hardware.armOne.configMotionAcceleration(6000, Constants.kTimeoutMs);

		/* zero the sensor */
		Hardware.armOne.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		//Hardware.armOne.configForwardSoftLimitThreshold((int)(UnitConverter.convertMetersToTicks(1.6)), 0);
		//Hardware.armOne.configForwardSoftLimitEnable(true, 0);
 
    new Notifier(new Runnable(){
			public void run() {
				loop();				
			}

    }).startPeriodic(0.005);
    
  }


 public void loop(){
    switch(currState){
    case ZEROING:
      if(getArmEncoderValue() < Constants.kBottomTolerance){
        updateState(ArmState.ZEROED);
        stop();
      }
      else if(Math.abs(getArmEncoderValue() - setpoint) < Constants.kBottomTolerance){
        updateState(ArmState.ZEROED);
        stop();
      }
      
        break;
    case SETPOINT: 
      if(Math.abs(getArmEncoderValue() - setpoint) < Constants.kSetpointTolerance){
        hold();
        updateState(ArmState.HOLD);
      }
        break;
    case ZEROED:
      break;
    case HOLD:
      hold();
      break;
    case MANUAL:
      manualControl(Robot.oi.getArmThrottle());
      break;
    }
  }

  public double getArmEncoderValue() {
    return Hardware.armOne.getSelectedSensorPosition();
  }


  public void hold() {
    Hardware.armOne.set(ControlMode.PercentOutput, Constants.kHoldVoltage);
  }

  public void manualControl(double armThrottle){
    Hardware.armOne.set(ControlMode.PercentOutput, armThrottle * 0.5);
  }

  public void updateState(ArmState state){
    currState = state;
  }

  public void stop(){
    Hardware.armOne.set(ControlMode.PercentOutput, 0);
  }

  public void setArmSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
    currState = ArmState.SETPOINT;
    Hardware.armOne.set(ControlMode.Position, setpoint);
  }

  public void zeroArm(String direction) {

    if(direction.equalsIgnoreCase("Forward")){
      setpoint = 0;
      currState = ArmState.ZEROING;
    }
    else if(direction.equalsIgnoreCase("Backward")){
      setpoint = Constants.kMaxEncoderValue;
      currState = ArmState.ZEROING;
    }
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
