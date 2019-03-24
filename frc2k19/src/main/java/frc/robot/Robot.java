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
import frc.robot.subsystems.LedStrip;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;

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
  public static LedStrip ledStrip;
  public static String lastSend = "nothing";
  public static I2C arduino = new I2C(I2C.Port.kOnboard, 128);
  public static boolean[] previousGiveBoolean;
  public static DigitalOutput port1;
  public static DigitalOutput port2;
  public static DigitalOutput port3;
  public int counter;
  public int width = 10;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public static void UpdateLEDs(String WriteString)
	{
    if(WriteString.equals(lastSend)) return;
    lastSend = WriteString;
		char[] CharArray = WriteString.toCharArray();
		byte[] WriteData = new byte[CharArray.length];
    for (int i = 0; i < CharArray.length; i++) 
    {
			WriteData[i] = (byte) CharArray[i];
		}
		arduino.writeBulk(WriteData, WriteData.length);
  }
  
  public static void UpdateLEDs(boolean input1, boolean input2, boolean input3)
	{
    if(previousGiveBoolean[0]!=input1 && previousGiveBoolean[1]!=input2
    && previousGiveBoolean[2]!=input3)
    {
      port1.set(input1);
      port2.set(input2);
      port3.set(input3);
      previousGiveBoolean[0]=input1;
      previousGiveBoolean[1]=input2;
      previousGiveBoolean[2]=input3;
    }
  }

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
    previousGiveBoolean = new boolean[3];;
		port1 = new DigitalOutput(7);
		port2 = new DigitalOutput(8);
		port3 = new DigitalOutput(9);
		previousGiveBoolean[1] = true;
    ledStrip = new LedStrip();

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
    Robot.UpdateLEDs("DISABLED");
    Robot.UpdateLEDs(false, false, true);
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

    Robot.UpdateLEDs("DRIVING");
    Robot.UpdateLEDs(false, false, false);
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