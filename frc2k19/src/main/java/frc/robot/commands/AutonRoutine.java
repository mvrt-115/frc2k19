/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Robot.AutonGoal;
import frc.robot.Robot.RobotStartLocation;
import frc.robot.Robot.RobotState;

public class AutonRoutine extends InstantCommand {
  
  RobotStartLocation location;
  AutonGoal goal;
  int autonStage;

  String path1, path2, path3;

  public AutonRoutine(RobotStartLocation location, AutonGoal goal, int stage) {
    this.location = location;
    this.goal = goal;
    autonStage = stage;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    if(Robot.currState != RobotState.AUTON)
      return;

    switch (location){
      case CENTER:

        if(goal == AutonGoal.LEFT_CARGOSHIP)
          path1 = "CenterTMiddleLeft";
        else
          path1 = "CenterToMiddleRight";
        
        break;
      case LEFT_BOTTOM:

        if(goal == AutonGoal.LEFT_CARGOSHIP){
          path1 = "LeftBottomToLeft1";
          path2 = "Left1ToStation";
          path3 = "StationToLeft2";
        }else{
          path1 = "LeftBottomToRocket";
          path2 = "RocketToLeftStation";
          path3 = "StationToLeft2";
        } 

        break;
      case LEFT_TOP:

        if(goal == AutonGoal.LEFT_CARGOSHIP){
          path1 = "LeftTopToLeft1";
          path2 = "Left1ToStation";
          path3 = "StationToLeft2";
        }else{
          path1 = "LeftTopToRocket";
          path2 = "RocketToLeftStation";
          path3 = "StationToLeft2";
        } 

        break;
      case RIGHT_BOTTOM:

        if(goal == AutonGoal.LEFT_CARGOSHIP){
          path1 = "RightBottomToRight1";
          path2 = "Right1ToStation";
          path3 = "StationToRight2";
        }else{
          path1 = "RightBottomToRocket";
         path2 = "RocketToRightStation";
         path3 = "StationToRight2";
        } 

        break;
      case RIGHT_TOP:
        if(goal == AutonGoal.LEFT_CARGOSHIP){
          path1 = "RightTopToRight1";
          path2 = "Right1ToStation";
          path3 = "StationToRight2";
        }else{
          path1 = "RightTopToRocket";
          path2 = "RocketToRightStation";
         path3 = "StationToRight2";
        } 
       break;
    }

    new DriveToTarget(path1, path2, path3, autonStage).start();


  }

  protected void execute() {}

  protected void end() {}

  protected void interrupted() {
    end();
  }

}