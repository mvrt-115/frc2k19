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
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.IntakePanel;
import frc.robot.commands.ManualControl;
import frc.robot.commands.MoveToSetpoint;
import frc.robot.commands.OuttakeCargo;
import frc.robot.commands.OuttakePanel;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  Joystick driverJoystick;
  Joystick operatorJoystick;
 
  JoystickButton quickTurn;
  JoystickButton toggleManual;
  
  POVButton intakeCargo;
  POVButton outtakeCargo;
  POVButton intakePanel;
  POVButton outtakePanel;

  JoystickButton zero;
  JoystickButton cargoShipFront;
  JoystickButton cargoShipBack;
  JoystickButton cargoRocketBack;
  JoystickButton cargoGroundIntake;


  public OI () {
    driverJoystick = new Joystick(1);
    operatorJoystick = new Joystick(0);

    zero = new JoystickButton(operatorJoystick, 1);   //A Button
    cargoShipFront = new JoystickButton(operatorJoystick, 3);
    cargoShipBack = new JoystickButton(operatorJoystick, 4);
    cargoGroundIntake = new JoystickButton(operatorJoystick, 5);
    cargoRocketBack = new JoystickButton(operatorJoystick, 2);
    toggleManual = new JoystickButton(operatorJoystick, 8);

    quickTurn = new JoystickButton(driverJoystick, 5);

    intakePanel = new POVButton(driverJoystick, 0);
    outtakePanel = new POVButton(driverJoystick, 180);
    intakeCargo = new POVButton(operatorJoystick, 0);
    outtakeCargo = new POVButton(operatorJoystick, 180);

    
    zero.whenPressed(new MoveToSetpoint(Constants.kZero));
    toggleManual.whenPressed(new ManualControl());
    cargoShipFront.whenPressed(new MoveToSetpoint(Constants.kCargoShipFront));
    cargoShipBack.whenPressed(new MoveToSetpoint(Constants.kCargoShipBack));
    cargoRocketBack.whenPressed(new MoveToSetpoint(Constants.kCargoRocketBack));
    cargoGroundIntake.whenPressed(new MoveToSetpoint(Constants.kCargoIntakeLevel));
  
    intakeCargo.whenPressed(new IntakeCargo());
    outtakeCargo.whenPressed(new OuttakeCargo());
    intakePanel.whenPressed(new IntakePanel());
    outtakePanel.whenPressed(new OuttakePanel());
  }

  public boolean getIntakeCargo() {
    return intakeCargo.get();
  }

  public boolean getOuttakeCargo() {
    return outtakeCargo.get();
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

}
