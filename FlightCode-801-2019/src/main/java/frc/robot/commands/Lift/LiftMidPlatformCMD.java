/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
//used to lift the robot to the top platform ~25 inches
public class LiftMidPlatformCMD extends Command {
  public LiftMidPlatformCMD() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.lift);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.lift.lift(Constants.LiftMidPlatformPosition); //go to 12"
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.lift.encoderPos();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.lift.isDeltaPosition();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lift.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
