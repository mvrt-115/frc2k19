/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.commands.ExtendRelease;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.IntakeHatchGround;
import frc.robot.commands.IntakePanel;
import frc.robot.commands.LedStripCommand;
import frc.robot.commands.ManualControl;
import frc.robot.commands.MoveToSetpoint;
import frc.robot.commands.OuttakeCargo;
import frc.robot.commands.OuttakeHatchGround;
import frc.robot.commands.OuttakePanel;
import frc.robot.commands.ShootCargo;
import frc.robot.commands.StowGroundIntake;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick driverJoystick;
  public Joystick operatorJoystick;

  JoystickButton quickTurn;
  JoystickButton toggleManual;
  JoystickButton visionControl;

  JoystickTrigger intakeCargo;
  JoystickTrigger outtakeCargo;
  JoystickButton shootCargo;

  JoystickButton intakePanel;
  JoystickTrigger outtakePanel;

  JoystickButton zero;
  JoystickButton cargoShipFront;
  JoystickButton cargoShipBack;
  JoystickButton cargoRocketBack;
  JoystickButton cargoGroundIntake;
  
  POVButton stowIntakeGround;
  POVButton intakeGround;
  POVButton outtakeGround;
  POVButton extendRelease;

  public OI() {
    driverJoystick = new Joystick(0);
    operatorJoystick = new Joystick(1);

    // arm
    zero = new JoystickButton(operatorJoystick, 1); // A Button
    cargoRocketBack = new JoystickButton(operatorJoystick, 2);
    cargoShipFront = new JoystickButton(operatorJoystick, 3);
    cargoShipBack = new JoystickButton(operatorJoystick, 4);
    cargoGroundIntake = new JoystickButton(operatorJoystick, 5);
    toggleManual = new JoystickButton(operatorJoystick, 8);

    // drive
    quickTurn = new JoystickButton(driverJoystick, 5);
    visionControl = new JoystickButton(driverJoystick, 6);

    // hatch intake
    intakePanel = new JoystickButton(operatorJoystick, 6);
    outtakePanel = new JoystickTrigger(operatorJoystick, 3);

    intakeCargo = new JoystickTrigger(driverJoystick, 3);
    outtakeCargo = new JoystickTrigger(driverJoystick, 2);
    shootCargo = new JoystickButton(driverJoystick, 7);

    // ground intake
    stowIntakeGround = new POVButton(operatorJoystick, 0);
    intakeGround = new POVButton(operatorJoystick, 270);
    outtakeGround = new POVButton(operatorJoystick, 90);
    extendRelease = new POVButton(driverJoystick, 180);
    zero.whenPressed(new MoveToSetpoint(Constants.kZero));
    toggleManual.whenPressed(new ManualControl());
    cargoShipFront.whenPressed(new MoveToSetpoint(Constants.kCargoShipFront));
    cargoShipBack.whenPressed(new MoveToSetpoint(Constants.kCargoShipBack));
    cargoRocketBack.whenPressed(new MoveToSetpoint(Constants.kCargoRocketBack));
    cargoGroundIntake.whenPressed(new MoveToSetpoint(Constants.kCargoIntakeLevel));

    shootCargo.whenPressed(new ShootCargo());
    outtakeCargo.whenActive(new OuttakeCargo());
    outtakeCargo.whenActive(new LedStripCommand());
    intakeCargo.whenActive(new IntakeCargo());
    intakeCargo.whenActive(new LedStripCommand());

    intakeGround.whenPressed(new IntakeHatchGround());
    outtakeGround.whenPressed(new OuttakeHatchGround());
    stowIntakeGround.whenPressed(new StowGroundIntake());

    intakePanel.whenPressed(new IntakePanel());
    outtakePanel.whenActive(new OuttakePanel());
    extendRelease.whenPressed(new ExtendRelease());
  }

  

  public boolean getGroundIntake() {
    return intakeGround.get();
  }

  public boolean getGroundOuttake() {
    return outtakeGround.get();
  }

  public boolean getOuttakeCargo() {
    return outtakeCargo.get();
  }
  
  public boolean getIntakeCargo() {
    return intakeCargo.get();
  }
  
  public boolean getShootCargo() {
    return shootCargo.get();
  }

  public boolean getManual() {
    return toggleManual.get();
  }

  public double getThrottle() {
    return driverJoystick.getRawAxis(5);
  }

  public double getWheel() {
    return driverJoystick.getRawAxis(0);
  }

  public boolean getQuickTurn() {
    return quickTurn.get();
  }

  public double getArmThrottle() {
    return operatorJoystick.getRawAxis(1);
  }

  public boolean getVisionTurn() {
    return visionControl.get();
  }

}
