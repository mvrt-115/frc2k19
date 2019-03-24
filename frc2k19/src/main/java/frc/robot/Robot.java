/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

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
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.PanelIntake;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Drivetrain drivetrain;
  public static Arm arm;
  public static OI oi;
  public static CargoIntake cargoIntake;
  public static PanelIntake panelIntake;
  public static GroundIntake groundIntake;
  public static Compressor c;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI(); 
    drivetrain = new Drivetrain();
    arm = new Arm();
    cargoIntake = new CargoIntake();
    panelIntake = new PanelIntake();
    groundIntake = new GroundIntake();

    Hardware.armOne.setNeutralMode(NeutralMode.Coast);
    Hardware.armTwo.setNeutralMode(NeutralMode.Coast);
    Hardware.armThree.setNeutralMode(NeutralMode.Coast);
    Hardware.armFour.setNeutralMode(NeutralMode.Coast);

    CameraServer.getInstance().startAutomaticCapture();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
 //   SmartDashboard.putNumber("Arm MotorOutput", Hardware.armOne.getMotorOutputPercent());
  //  SmartDashboard.putNumber("Arm Setpoint", arm.setpoint);
    SmartDashboard.putNumber("Arm Encoder Value", arm.getArmEncoderValue());
    SmartDashboard.putBoolean("Limit Switch", arm.hallEffect1.get());
//    SmartDashboard.putBoolean("Back Hall Effect", arm.hallEffect2.get());
 //   SmartDashboard.putNumber("Limelight", drivetrain.getAngle());
    SmartDashboard.putNumber("Drivetrain Encoder", drivetrain.getleftEncoderPosition());
 //   SmartDashboard.putNumber("2 Drivetrain Encoder", drivetrain.getRightEncoderPosition());
   // SmartDashboard.putNumber("Right Output ", Hardware.frontRight.getAppliedOutput());
   // SmartDashboard.putNumber("Left Output", Hardware.frontLeft.getAppliedOutput());
    SmartDashboard.putNumber("Ground Encoder Value", Hardware.groundPivot.getSelectedSensorPosition());
  //  SmartDashboard.putNumber("Ground pivot output", Hardware.groundPivot.getMotorOutputPercent());
   // SmartDashboard.putNumber("Ground pivot error", Hardware.groundPivot.getClosedLoopError());
    SmartDashboard.putNumber("Arm Output", Hardware.armOne.getMotorOutputPercent());

    SmartDashboard.putBoolean("Cargo BreakBeam", Robot.cargoIntake.breakbeam.get());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    Hardware.groundPivot.set(ControlMode.PercentOutput, 0.0);
  }


  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    
    arm.currState = ArmState.ZEROED;
    arm.setpoint = 0;
    Hardware.armOne.setSelectedSensorPosition(0);
    
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    Hardware.frontLeftEncoder.setPosition(0);
    Hardware.frontRightEncoder.setPosition(0);
    Hardware.backLeftEncoder.setPosition(0);
    Hardware.backRightEncoder.setPosition(0);

    Hardware.groundPivot.setSelectedSensorPosition(0);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
   // SmartDashboard.putNumber("ERROR", (arm.setpoint) - arm.getArmEncoderValue());
   // SmartDashboard.putNumber("Talon Error", Hardware.armOne.getClosedLoopError());
    Scheduler.getInstance().run(); 
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}