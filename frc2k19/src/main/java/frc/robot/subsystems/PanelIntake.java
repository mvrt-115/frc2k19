/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.commands.FlashLimelight;


public class PanelIntake extends Subsystem {

  DigitalInput limitSwitch;


  public PanelIntake() {
    Hardware.claw = new DoubleSolenoid(1, 1, 0);
    Hardware.activeRelease = new DoubleSolenoid(1, 3, 2);
    limitSwitch = new DigitalInput(4);
  }

  public void intakePanel() {

    if (Hardware.activeRelease.get() == Value.kReverse){
      Hardware.claw.set(Value.kReverse);
      Hardware.activeRelease.set(Value.kForward);
    }
    else if (Hardware.claw.get() == Value.kReverse){
      Hardware.claw.set(Value.kForward);
      Robot.arm.setArmSetpoint(1500);
      new FlashLimelight().start();      
    }else{
      Hardware.activeRelease.set(Value.kReverse);
      Robot.arm.setArmSetpoint(0);
    }
  }

  public void outtakePanel(int state) {

    if(state ==1) 
       Hardware.activeRelease.set(Value.kForward);
    else if(state ==2)
      Hardware.claw.set(Value.kReverse);
    else if(state ==3)
       Hardware.activeRelease.set(Value.kReverse);
    else 
      Hardware.claw.set(Value.kForward);
   }

   public void loop(){
      if(Hardware.claw.get() == Value.kReverse && limitSwitch.get()){
    //    Hardware.claw.set(Value.kForward);
     //   Robot.arm.setArmSetpoint(1500);
      //  new FlashLimelight().start();    
      }

   }

   public void initDefaultCommand() {}
}
