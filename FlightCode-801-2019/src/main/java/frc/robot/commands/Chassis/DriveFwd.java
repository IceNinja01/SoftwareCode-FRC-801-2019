/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveFwd extends Command {
  private double setPoint;
  private double velocity;
  private double angle;

  //Used to drive by angle and velocity, stops at setpoint
  public DriveFwd(double angle, double velocity, double setPoint) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.chassis);
    this.angle = angle;
    this.velocity = velocity;
    this.setPoint = setPoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.chassis.motorDrive_CMD(angle, velocity);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.chassis.isDistance(setPoint);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.chassis.stop();

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

}
