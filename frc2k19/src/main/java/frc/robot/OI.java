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
  JoystickButton intakeCargo;
  JoystickButton outtakeCargo;
  POVButton intakePanel;
  POVButton outtakePanel;


  JoystickButton zero;
  JoystickButton hatchBackwards;
  JoystickButton cargoShipFront;
  JoystickButton cargoShipBack;
  JoystickButton cargoRocketFront;
  JoystickButton cargoRocketBack;
  JoystickButton cargoGroundIntake;

  JoystickButton toggleManual;

  public OI () {
    driverJoystick = new Joystick(1);
    operatorJoystick = new Joystick(0);

    zero = new JoystickButton(operatorJoystick, 1);   //A Button
   // cargoRocketFront = new JoystickButton(operatorJoystick, 2);
   // cargoShipFront = new JoystickButton(operatorJoystick, 3);
    cargoShipBack = new JoystickButton(operatorJoystick, 4);
    cargoRocketBack = new JoystickButton(operatorJoystick, 5);
    hatchBackwards = new JoystickButton(operatorJoystick, 6);
    cargoGroundIntake = new JoystickButton(operatorJoystick, 7);
    
    intakePanel = new POVButton(driverJoystick, 90);
    outtakePanel = new POVButton(driverJoystick, 270);
    
    quickTurn = new JoystickButton(driverJoystick, 5);
    intakeCargo = new JoystickButton(operatorJoystick, 5);
    outtakeCargo = new JoystickButton(operatorJoystick, 6);

    toggleManual = new JoystickButton(operatorJoystick, 8);
    
    zero.whenPressed(new MoveToSetpoint(Constants.kZero));
    toggleManual.whenPressed(new ManualControl());
   // cargoRocketFront.whenPressed(new MoveToSetpoint(Constants.kCargoRocketFront));
    //cargoRocketBack.whenPressed(new MoveToSetpoint(Constants.kCargoRocketBack));
    //cargoShipFront.whenPressed(new MoveToSetpoint(Constants.kCargoShipFront));
    cargoShipBack.whenPressed(new MoveToSetpoint(Constants.kCargoShipBack));
    hatchBackwards.whenPressed(new MoveToSetpoint(Constants.kHatchBack));
    cargoGroundIntake.whenPressed(new MoveToSetpoint(Constants.kCargoIntakeLevel));
  
    intakeCargo.whenPressed(new IntakeCargo());
    outtakeCargo.whenPressed(new OuttakeCargo());
    //intakePanel.whenPressed(new IntakeCargo());
    //outtakePanel.whenPressed(new OuttakeCargo());
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
