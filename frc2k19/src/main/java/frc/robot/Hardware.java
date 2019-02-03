/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

/**
 * Add your docs here.
 */
public class Hardware {

    public static CANSparkMax frontLeft;
    public static CANSparkMax frontRight;
    public static CANSparkMax backLeft;
    public static CANSparkMax backRight;

    public static TalonSRX armOne;
    public static TalonSRX armTwo;
    public static TalonSRX armThree;
    public static TalonSRX armFour;
}
