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

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.commands.IntakeHatchGround;

/**
 * Add your docs here.
 */
public class GroundIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public enum IntakeState {
    ZEROED, ZEROING, SETPOINT, HOLD
  };

  public double setpoint;
  public IntakeState currState = IntakeState.ZEROED;

  public GroundIntake() {

    setpoint = 0;

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

    new Notifier(new Runnable() {

      public void run() {
        loop();
      }

    }).startPeriodic(0.005);

  }

  public void loop() {

    switch (currState) {
      case ZEROING:
        SmartDashboard.putString("Intake State", "zeroing");
        if (getIntakeEncoderValue() < Constants.kStowTolerance) {
          stop();
        } else {
          Hardware.groundPivot.set(ControlMode.MotionMagic, setpoint);
        }
        break;
      case SETPOINT:   
        if(setpoint == Constants.kCargoIntakeLevel){
          if (getIntakeEncoderValue() > Constants.kGroundTolerance) {
            updateState(IntakeState.ZEROED);
          } else {
            SmartDashboard.putString("Intake State", "setpoint");
            Hardware.groundPivot.set(ControlMode.MotionMagic, setpoint);
          }
        }
        else{
          SmartDashboard.putString("Intake State", "setpoint");
          Hardware.groundPivot.set(ControlMode.MotionMagic, setpoint);
        }
        break;
      case ZEROED:
        SmartDashboard.putString("Intake State", "zeroed");
        stop();
        break;
      case HOLD:
        SmartDashboard.putString("Intake State", "hold");
        // hold();
        break;
  
      }
  }

  public void intakeHatch(){
    Hardware.groundIntake.set(ControlMode.PercentOutput, -0.6);
  }

  public void outtakeHatch() {
    Hardware.groundIntake.set(ControlMode.PercentOutput, 0.6);
  }

  public double getIntakeEncoderValue() {
    return Hardware.groundPivot.getSelectedSensorPosition(0);
  }

  public void updateState(IntakeState state) {
    currState = state;
  }

  public void stop() {
    Hardware.groundIntake.set(ControlMode.PercentOutput, 0);
  }

  public void setIntakeSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
    currState = IntakeState.SETPOINT;
  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new IntakeHatchGround());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
