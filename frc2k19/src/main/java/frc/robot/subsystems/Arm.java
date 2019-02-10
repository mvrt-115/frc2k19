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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {

  public enum ArmState {
    ZEROED, ZEROING, SETPOINT, HOLD, MANUAL
  };

  public double setpoint;
  public boolean isInverted;
  public DigitalInput hallEffect1;
  public DigitalInput hallEffect2;

  public ArmState currState = ArmState.ZEROED;

  public Arm() {

    setpoint = 0;
    isInverted = false;

    // 36 anbd 33 have encoders
    Hardware.armOne = new TalonSRX(Constants.kArmOne);
    Hardware.armTwo = new TalonSRX(Constants.kArmTwo);
    Hardware.armThree = new TalonSRX(Constants.kArmThree);
    Hardware.armFour = new TalonSRX(Constants.kArmFour);

    hallEffect1 = new DigitalInput(8);
    hallEffect2 = new DigitalInput(9);
    

    Hardware.armTwo.follow(Hardware.armOne);
    Hardware.armThree.follow(Hardware.armOne);
    Hardware.armFour.follow(Hardware.armOne);

    Hardware.armOne.setInverted(true);
    Hardware.armThree.setInverted(true);
    Hardware.armTwo.setInverted(false);
    Hardware.armFour.setInverted(false);  
    //Hardware.armTwo.setInverted(true);

    Hardware.armOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    Hardware.armOne.setSensorPhase(false);

    Hardware.armOne.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    Hardware.armOne.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    Hardware.armOne.configNominalOutputForward(0, Constants.kTimeoutMs);
    Hardware.armOne.configNominalOutputReverse(0, Constants.kTimeoutMs);

    Hardware.armOne.configPeakOutputForward(0.7, Constants.kTimeoutMs);
    Hardware.armOne.configPeakOutputReverse(-0.7, Constants.kTimeoutMs);

    Hardware.armOne.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    Hardware.armOne.config_kF(0, Constants.kArmF, Constants.kTimeoutMs);
    Hardware.armOne.config_kP(0, Constants.kArmP, Constants.kTimeoutMs);
    Hardware.armOne.config_kD(0, Constants.kArmD, Constants.kTimeoutMs);
    
    Hardware.armOne.configAllowableClosedloopError(Constants.kSlotIdx, 90, Constants.kTimeoutMs);

    Hardware.armOne.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    Hardware.armOne.configMotionAcceleration(6000, Constants.kTimeoutMs);

    Hardware.armOne.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    new Notifier(new Runnable() {

      public void run() {
        loop();
      }

    }).startPeriodic(0.005);

  }


  public void loop() {
  
    switch (currState) {
      case ZEROING:
        SmartDashboard.putString("armState", "zeroing");
        if (getArmEncoderValue() < Constants.kBottomTolerance) {
            stop();
        } 
        else if (!hallEffect1.get()) {
          Hardware.armOne.setSelectedSensorPosition(0);
          updateState(ArmState.ZEROED);
        }
        break;
      case SETPOINT:
        if(!hallEffect2.get()) {
          updateState(ArmState.ZEROED); 
          stop();
        }
        else {
          SmartDashboard.putString("armState", "setpoint");
          SmartDashboard.putNumber("armSetpoint", setpoint);
          Hardware.armOne.set(ControlMode.MotionMagic, setpoint);
        }
        break;
      case ZEROED:
        SmartDashboard.putString("armState", "zeroed");
        stop();
        break;
      case HOLD:
        SmartDashboard.putString("armState", "hold");
        break;
      case MANUAL:
        SmartDashboard.putString("armState", "manual");
        manualControl(0.5 * Robot.oi.getArmThrottle());
        break;
    }
  }  
  

  public double getArmEncoderValue() {
    return Hardware.armOne.getSelectedSensorPosition(0);
  }

  public void hold() {
    Hardware.armOne.set(ControlMode.PercentOutput, Constants.kHoldVoltage);
  }

  public void manualControl(double armThrottle) {
    Hardware.armOne.set(ControlMode.PercentOutput, armThrottle);
  }

  public void updateState(ArmState state) {
    currState = state;
  }

  public void stop() {
    Hardware.armOne.set(ControlMode.PercentOutput, 0);
  }

  public void setArmSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
    currState = ArmState.SETPOINT;
  }

  public void zeroArm() {
      setpoint = Constants.kZero;
      currState = ArmState.ZEROING;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}