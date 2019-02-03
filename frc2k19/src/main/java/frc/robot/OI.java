/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.MoveToSetpoint;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  Joystick driverJoystick;
  Joystick operatorJoystick;
  JoystickButton quickTurn;
  JoystickButton zeroForwards;
  JoystickButton zeroBackwards;
  JoystickButton hatchLow;
  JoystickButton cargoShip;
  JoystickButton cargoRocket;

  public OI () {
    driverJoystick = new Joystick(0);
    operatorJoystick = new Joystick(1);

    zeroForwards = new JoystickButton(operatorJoystick, 1);
    zeroBackwards = new JoystickButton(operatorJoystick, 2);
    hatchLow = new JoystickButton(operatorJoystick, 3);
    cargoShip = new JoystickButton(operatorJoystick, 1);

    quickTurn = new JoystickButton(driverJoystick, 5);
    

    zeroForwards.whenPressed(new MoveToSetpoint(Constants.kMinEncoderValue));
    zeroBackwards.whenPressed(new MoveToSetpoint(Constants.kMaxEncoderValue)); 
    hatchLow.whenPressed(new MoveToSetpoint(Constants.kHatchLow));
    cargoShip.whenPressed(new MoveToSetpoint(Constants.kCargoShip));
    cargoRocket.whenPressed(new MoveToSetpoint(Constants.kCargoRocket));
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
