/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class FollowProfile extends Command {
  
  ScheduledExecutorService scheduler;
	ScheduledFuture<?> motionFollower;
	TankModifier modifier;
	public Trajectory trajectory;
  public int counter;
  
  public FollowProfile(Waypoint[] points) {
    scheduler = Executors.newScheduledThreadPool(1);
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, Constants.TIME_STEP, Constants.MAX_VELOCITY, Constants.MAX_ACCEL, Constants.MAX_JERK);
		trajectory = Pathfinder.generate(points, config);
		counter=0;	
		modifier = new TankModifier(trajectory).modify(Constants.WHEELBASE_WIDTH);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    motionFollower = scheduler.scheduleAtFixedRate(new Runnable() {
			public void run() {
				Robot.drivetrain.updateMotionFollowing();
			}
			
		}, (int)(Constants.TIME_STEP*1000), (int)(Constants.TIME_STEP*1000), TimeUnit.MILLISECONDS);
		
		Hardware.leftFollower.setTrajectory(modifier.getLeftTrajectory());
		Hardware.rightFollower.setTrajectory(modifier.getRightTrajectory());
		Hardware.leftFollower.configureEncoder((int)(Robot.drivetrain.getleftEncoderPosition() * 42) , Constants.TICKS_PER_ROTATION, Constants.WHEEL_DIAMETER);
		Hardware.rightFollower.configureEncoder((int)(Robot.drivetrain.getRightEncoderPosition() *42), Constants.TICKS_PER_ROTATION, Constants.WHEEL_DIAMETER);
		 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
