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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

    // 36 and 33 have encoders
    Hardware.armOne = new TalonSRX(Constants.kArmOne);
    Hardware.armTwo = new TalonSRX(Constants.kArmTwo);
    Hardware.armThree = new TalonSRX(Constants.kArmThree);
    Hardware.armFour = new TalonSRX(Constants.kArmFour);

    hallEffect1 = new DigitalInput(0);
    hallEffect2 = new DigitalInput(1);

    Hardware.armTwo.follow(Hardware.armOne);
    Hardware.armThree.follow(Hardware.armOne);
    Hardware.armFour.follow(Hardware.armOne);

    if(Constants.kCompbot) {
      Hardware.armOne.setInverted(false);
      Hardware.armThree.setInverted(false);
      Hardware.armTwo.setInverted(true);
      Hardware.armFour.setInverted(true);
    }

    else {
      Hardware.armOne.setInverted(true);
      Hardware.armThree.setInverted(true);
      Hardware.armTwo.setInverted(false);
      Hardware.armFour.setInverted(false);

    }

    Hardware.armOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);

    Hardware.armOne.setSensorPhase(false);

    Hardware.armOne.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    Hardware.armOne.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    Hardware.armOne.configNominalOutputForward(0, Constants.kTimeoutMs);
    Hardware.armOne.configNominalOutputReverse(0, Constants.kTimeoutMs);

    Hardware.armOne.configPeakOutputForward(0.85, Constants.kTimeoutMs);
    Hardware.armOne.configPeakOutputReverse(-0.79, Constants.kTimeoutMs);

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
  // test

  public void loop() {

    if (!isInverted && getArmEncoderValue() > Constants.kMidEncoderValue) {
      isInverted = true;
      Robot.drivetrain.switchPipeline(1);
    }

    if (isInverted && getArmEncoderValue() < Constants.kMidEncoderValue) {
      isInverted = false;
      Robot.drivetrain.switchPipeline(0);
    }

    if(setpoint !=0 && setpoint !=Constants.kZero  && setpoint != 1500){
      Hardware.activeRelease.set(Value.kReverse);  
      Hardware.claw.set(Value.kForward);    
    }

    switch (currState) {
    case ZEROING:
      SmartDashboard.putString("Arm State", "zeroing");
     
      if (!hallEffect1.get()) {
        updateState(ArmState.ZEROED);
        Hardware.armOne.setSelectedSensorPosition(0);
      } else if (getArmEncoderValue() < Constants.kBottomTolerance) {
     //    Hardware.armOne.set(ControlMode.PercentOutput, 0.038);
          stop();
      } else {
        Hardware.armOne.set(ControlMode.MotionMagic, setpoint);
      }
      break;
    case SETPOINT:   
      if(setpoint == Constants.kCargoIntakeLevel){

         if (getArmEncoderValue() > Constants.kBackTolerance) {
          updateState(ArmState.ZEROED);
        } else {
          SmartDashboard.putString("Arm State", "setpoint");
          Hardware.armOne.set(ControlMode.MotionMagic, setpoint);
        }
      }
      else{
        SmartDashboard.putString("Arm State", "setpoint");
        Hardware.armOne.set(ControlMode.MotionMagic, setpoint);
      }
      break;
    case ZEROED:
      SmartDashboard.putString("Arm State", "zeroed");
      stop();
      break;
    case HOLD:
      SmartDashboard.putString("Arm State", "hold");
      // hold();
      break;
    case MANUAL:
      SmartDashboard.putString("Arm State", "manual");
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
    Hardware.armOne.set(ControlMode.PercentOutput, -armThrottle);
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

  public void initDefaultCommand() {
  }
}
