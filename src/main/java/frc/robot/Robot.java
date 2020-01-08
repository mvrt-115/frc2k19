/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PanelIntake;


public class Robot extends TimedRobot {

  public static Drivetrain drivetrain;
  public static Arm arm;
  public static OI oi;
  public static CargoIntake cargoIntake;
  public static PanelIntake panelIntake;
  public static Climber climber;
  public static Compressor c;

  public static RobotState currState;
  public static String autonPath;

  SendableChooser<Integer> autonStart = new SendableChooser<>();
  SendableChooser<Integer> autonEnd = new SendableChooser<>();

  public static String lastSend;
	public static I2C arduino;

  public enum RobotState {
    DISABLED, TELEOP, AUTON
  };

  public void robotInit() {
    drivetrain = new Drivetrain();
    arm = new Arm();
    cargoIntake = new CargoIntake();
    panelIntake = new PanelIntake();
    climber = new Climber();
    oi = new OI(); 

    CameraServer.getInstance().startAutomaticCapture();

    Hardware.armOne.setNeutralMode(NeutralMode.Coast);
    Hardware.armTwo.setNeutralMode(NeutralMode.Coast);
    Hardware.armThree.setNeutralMode(NeutralMode.Coast);
    Hardware.armFour.setNeutralMode(NeutralMode.Coast);

    
 //   autonStart.setName("Auton Start Position");
    autonStart.setDefaultOption("Front Center", 1);
    autonStart.addOption("Front Left", 2);
    autonStart.addOption("Front Right", 3);
    autonStart.addOption("Back Left", 4);
    autonStart.addOption("Back Right", 5);
    autonStart.addOption("None", 6);

    autonEnd.setDefaultOption("Rocket", 1);
    autonEnd.addOption("Center Left", 2);
    autonEnd.addOption("Center Right", 3);
    autonEnd.addOption("Right 1", 4);
    autonEnd.addOption("Right 2", 5);
    autonEnd.addOption("Left 1", 6);
    autonEnd.addOption("Left 2", 7);
  

    SmartDashboard.putData("Autonomous Start", autonStart);
    SmartDashboard.putData("Autonomous End", autonEnd);

    currState = RobotState.DISABLED;
    autonPath = "";
  }


  @Override
  public void robotPeriodic() {


    
    cargoIntake.log();
    arm.log();
    drivetrain.log();
    panelIntake.log();
   
    climber.log();

    
    int autonStartLocation = autonStart.getSelected();
    int autonEndLocation = autonEnd.getSelected();

    switch(autonStartLocation){

      case 1:
        if(autonEndLocation == 2){
          autonPath = "BotCenter-FrontLeft"; 
        }else if(autonEndLocation == 3){
          autonPath = "BotCenter-FrontRight";
        }
        break;
      case 2:
        if(autonEndLocation == 2){
          autonPath = "BotLeft-FrontLeft";
        }
        else if(autonEndLocation == 1){
          autonPath = "BotLeft-Rocket";
        }else if(autonEndLocation == 6){
          autonPath = "BotLeft-Left1";
        }
        break;
      case 3:
        if(autonEndLocation == 1){
          autonPath = "BotRight-Rocket";
        }else if(autonEndLocation == 3){
          autonPath = "BotRight-FrontRight";
        }else if(autonEndLocation == 4){
          autonPath = "BotRight-Right1";
        }
        break;
      case 4:
        
        if(autonEndLocation == 1){
          autonPath = "TopLeft-Rocket";
        }else if(autonEndLocation == 2){
          autonPath = "TopLeft-FrontLeft";
        }else if(autonEndLocation == 6){
          autonPath = "TopLeft-Left1";
        }
        break;
    
       case 5:
        if(autonEndLocation == 1){
          autonPath = "TopRight-Rocket";
        }else if(autonEndLocation ==3){
          autonPath = "TopRight-FrontRight";
        }else if(autonEndLocation == 4){
          autonPath = "TopRight-Right1";
        }
       break;
      
      case 6:
        autonPath = "";
      break;
   }


    SmartDashboard.putString("AutonPath", autonPath);
  }

 
  public void disabledInit() {
    currState = RobotState.DISABLED;
//    updateLEDs("DISABLED");
  /*  Hardware.frontLeft.setIdleMode(IdleMode.kCoast);
		Hardware.backLeft.setIdleMode(IdleMode.kCoast);
		Hardware.frontRight.setIdleMode(IdleMode.kCoast);
    Hardware.backRight.setIdleMode(IdleMode.kCoast);
    */

  }


  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

 
  public void autonomousInit() {
    currState = RobotState.AUTON;

    drivetrain.navX.zeroYaw();

    Hardware.frontLeftEncoder.setPosition(0);
    Hardware.frontRightEncoder.setPosition(0);
    Hardware.backLeftEncoder.setPosition(0);
    Hardware.backRightEncoder.setPosition(0);

    arm.firstTime = true;
    arm.currState = ArmState.ZEROED;
    arm.setpoint = 0;

    Hardware.armOne.setSelectedSensorPosition(0);
    Hardware.rightClimb.setSelectedSensorPosition(0);
    
    Hardware.claw.set(Value.kForward);
    Hardware.slider.set(Value.kReverse);
   // updateLEDs("OTHER");


}

  
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  public void teleopInit() {
    currState = RobotState.TELEOP;
  //  updateLEDs("DRIVING");
  }

  public void teleopPeriodic() {
    Scheduler.getInstance().run();


  }

  public void testPeriodic() {}

  public String getAutonPath(){
    return autonPath;
  }

}