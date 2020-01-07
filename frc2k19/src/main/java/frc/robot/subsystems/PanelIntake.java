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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.commands.FlashLimelight;


public class PanelIntake extends Subsystem {

  public DigitalInput limitSwitch;

  public PanelIntake() {
    Hardware.claw = new DoubleSolenoid(0, 5, 2);
    Hardware.slider = new DoubleSolenoid(0, 6, 1);  // 6 1
    limitSwitch = new DigitalInput(4);
    Hardware.pancakeLeft = new DoubleSolenoid(3, 4);
    Hardware.pancakeRight = new DoubleSolenoid(0, 7);
    // 0 7
    // 3 4

  }

  public void intakePanel() {

    if(Hardware.slider.get() == Value.kReverse && Hardware.claw.get() == Value.kForward){
      Hardware.claw.set(Value.kReverse);
      Hardware.slider.set(Value.kForward);
    }else if(Hardware.claw.get() == Value.kReverse && Hardware.slider.get() == Value.kForward){
      if(Robot.arm.setpoint != Constants.kCargoIntakeLevel)
          Robot.arm.setArmSetpoint(2000);
        Hardware.claw.set(Value.kForward);
        new FlashLimelight().start();
    }else {
        if(Robot.arm.setpoint == 2000)
          Robot.arm.zeroArm();          
        
       Hardware.slider.set(Value.kReverse); 
    }
    // if(Hardware.claw.get() == Value.kForward && Hardware.slider.get() == Value.kForward )

   /* if (Hardware.slider.get() == Value.kReverse && Robot.arm.setpoint != 2000){
      Hardware.claw.set(Value.kReverse);
      Hardware.slider.set(Value.kForward);
    }
    else if (Hardware.claw.get() == Value.kReverse){
      Hardware.claw.set(Value.kForward);
      
      if(Robot.arm.setpoint != Constants.kCargoIntakeLevel)
        Robot.arm.setArmSetpoint(2000);
    
    }else if(Hardware.claw.get() == Value.kForward &&    Robot.arm.setpoint == 2000){
      new FlashLimelight().start();    
      Robot.arm.zeroArm();
    }else{
      Hardware.slider.set(Value.kReverse);
      Robot.arm.zeroArm();
     // new FlashLimelight().start();      

    }

    */
  }

  public void outtakePanel(int state) {

    if(state ==1) 
       Hardware.slider.set(Value.kForward);
    else if(state ==2)
      Hardware.claw.set(Value.kReverse);
    else if(state ==3)
       Hardware.slider.set(Value.kReverse);
    else 
      Hardware.claw.set(Value.kForward);
   }

   public void initDefaultCommand() {}


   public void log(){
    
    SmartDashboard.putBoolean("Hatch Detected", limitSwitch.get());
   } 
}
