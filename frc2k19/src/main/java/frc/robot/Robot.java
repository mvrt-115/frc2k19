/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.PanelIntake;


public class Robot extends TimedRobot {
  public static Drivetrain drivetrain;
  public static Arm arm;
  public static OI oi;
  public static CargoIntake cargoIntake;
  public static PanelIntake panelIntake;
  //public static GroundIntake groundIntake;
  public static Climber climber;
  public static Compressor c;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public void robotInit() {
    oi = new OI(); 
    drivetrain = new Drivetrain();
    arm = new Arm();
    cargoIntake = new CargoIntake();
    panelIntake = new PanelIntake();
    //groundIntake = new GroundIntake();
    climber = new Climber();

    Hardware.armOne.setNeutralMode(NeutralMode.Coast);
    Hardware.armTwo.setNeutralMode(NeutralMode.Coast);
    Hardware.armThree.setNeutralMode(NeutralMode.Coast);
    Hardware.armFour.setNeutralMode(NeutralMode.Coast);
    //setting resolution here for usbcam
    //UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
    //cam.setResolution(640, 480);

    //original camera intialization code
    CameraServer.getInstance().startAutomaticCapture();


    SmartDashboard.putString("Enter path selection: ", "default");
    drivetrain.k_path_name = SmartDashboard.getString("Enter path selection: ", "default");

  }

  @Override
  public void robotPeriodic() {
    cargoIntake.log();
    arm.log();
    
    SmartDashboard.putNumber("Throttle", Robot.oi.getThrottle());
    SmartDashboard.putNumber("Throttle 2", Robot.oi.getWheel());
    SmartDashboard.putNumber("Drivetrain Encoder", drivetrain.getleftEncoderPosition());    
    SmartDashboard.putNumber("Climber Encoder", Hardware.rightClimb.getSelectedSensorPosition());
  }

 
  public void disabledInit() {
    drivetrain.navX.zeroYaw();
    //Hardware.groundPivot.set(ControlMode.PercentOutput, 0.0);
  }


  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

 
  public void autonomousInit() {

    drivetrain.k_path_name = SmartDashboard.getString("Enter path selection: ", "default");

    m_autonomousCommand = m_chooser.getSelected();

    Hardware.frontLeftEncoder.setPosition(0);
    Hardware.frontRightEncoder.setPosition(0);
    Hardware.backLeftEncoder.setPosition(0);
    Hardware.backRightEncoder.setPosition(0);

    arm.firstTime = true;
    arm.currState = ArmState.ZEROED;
    arm.setpoint = 0;
    Hardware.armOne.setSelectedSensorPosition(0);

    Hardware.rightClimb.setSelectedSensorPosition(0);
    
    
  }

  
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  public void teleopInit() {}

  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  public void testPeriodic() {}
}