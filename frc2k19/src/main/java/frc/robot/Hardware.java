/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import jaci.pathfinder.followers.EncoderFollower;

/**
 * Add your docs here.
 */
public class Hardware {

    public static CANSparkMax frontLeft;
    public static CANSparkMax frontRight;
    public static CANSparkMax backLeft;
    public static CANSparkMax backRight;

    public static CANEncoder frontLeftEncoder;
    public static CANEncoder frontRightEncoder;
    public static CANEncoder backLeftEncoder;
    public static CANEncoder backRightEncoder;

    public static EncoderFollower leftFollower;
    public static EncoderFollower rightFollower;

    public static TalonSRX armOne;
    public static TalonSRX armTwo;
    public static TalonSRX armThree;
    public static TalonSRX armFour;

    public static TalonSRX cargoIntakeTop;
    public static TalonSRX cargoIntakeBottom;

    public static TalonSRX leftClimb;
    public static TalonSRX rightClimb;

    public static TalonSRX rightClimbRoller;
    public static TalonSRX leftClimbRoller;

    public static DoubleSolenoid claw;
    public static DoubleSolenoid slider;

    public static DoubleSolenoid pancakeLeft;
    public static DoubleSolenoid pancakeRight;
}
