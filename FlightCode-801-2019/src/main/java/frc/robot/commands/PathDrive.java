/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import frc.robot.RobotMap;
import frc.robot.subsystems.Chassis;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.SwerveModifier;


/**
 * An example command.  You can replace me with your own command.
 */
public class PathDrive extends Command {
  private PathfinderFRC path;
  private Trajectory trajectory;
  private Trajectory fl;
  private Trajectory fr;
  private Trajectory bl;
  private Trajectory br;

  private EncoderFollower flFollower;
  private EncoderFollower frFollower;
  private EncoderFollower blFollower;
  private EncoderFollower brFollower;
  private double kP, kI, kD, maxVel, kAcc;
  private double frOutput, flOutput, blOutput, brOutput, desiredHeading;
  private double frHeading, flHeading, blHeading, brHeading;

  public PathDrive(String name, double kP, double kI, double kD, double maxVel, double kAcc) throws IOException {
    trajectory = path.getTrajectory(name);
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.maxVel = maxVel;
    this.kAcc = kAcc;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Wheelbase Width = 0.5m, Wheelbase Depth = 0.6m, Swerve Mode = Default
    SwerveModifier modifier = new SwerveModifier(trajectory).modify(0.7239, 0.7239, SwerveModifier.Mode.SWERVE_DEFAULT);

    // Swerve Drive Modifiers
    fl = modifier.getFrontLeftTrajectory();
    fr = modifier.getFrontRightTrajectory();
    bl = modifier.getBackLeftTrajectory();
    br = modifier.getBackRightTrajectory();

    frFollower = new EncoderFollower(modifier.getFrontRightTrajectory());   // Front Left wheel
    flFollower = new EncoderFollower(modifier.getFrontLeftTrajectory());   // Front Left wheel
    blFollower = new EncoderFollower(modifier.getBackLeftTrajectory());   // Front Left wheel
    brFollower = new EncoderFollower(modifier.getBackRightTrajectory());   // Front Left wheel

    // Encoder Position is the current, cumulative position of your encoder. If you're using an SRX, this will be the
    // 'getEncPosition' function.
    // 1000 is the amount of encoder ticks per full revolution
    // Wheel Diameter is the diameter of your wheel in meters
   
    frFollower.configureEncoder((int) Robot.chassis.rightFrontPod.getPosition(), 1, 0.1016);
    flFollower.configureEncoder((int) Robot.chassis.leftFrontPod.getPosition(), 1, 0.1016);
    blFollower.configureEncoder((int) Robot.chassis.leftBackPod.getPosition(), 1, 0.1016);
	  brFollower.configureEncoder((int) Robot.chassis.rightBackPod.getPosition(), 1, 0.1016);

    // The first argument is the proportional gain. Usually this will be quite high
    // The second argument is the integral gain. This is unused for motion profiling
    // The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
    // The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the 
    //      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
    // The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker
    frFollower.configurePIDVA(kP, kI, kD, 1 / maxVel, kAcc);
    flFollower.configurePIDVA(kP, kI, kD, 1 / maxVel, kAcc);
    blFollower.configurePIDVA(kP, kI, kD, 1 / maxVel, kAcc);
    brFollower.configurePIDVA(kP, kI, kD, 1 / maxVel, kAcc);

    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Calculate error and PID outputs
    frOutput = frFollower.calculate((int) Robot.chassis.rightFrontPod.getPosition());
    flOutput = flFollower.calculate((int) Robot.chassis.leftFrontPod.getPosition());
    blOutput = blFollower.calculate((int) Robot.chassis.leftBackPod.getPosition());
    brOutput = brFollower.calculate((int) Robot.chassis.rightBackPod.getPosition());
    frHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(frFollower.getHeading()));    // Bound to -180..180 degrees
    flHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(flFollower.getHeading()));    // Bound to -180..180 degrees
    blHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(blFollower.getHeading()));    // Bound to -180..180 degrees
    brHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(brFollower.getHeading()));    // Bound to -180..180 degrees
    //set speed for Swerve PODs
    Robot.chassis.rightFrontPod.setSpeed(frOutput);
    Robot.chassis.leftFrontPod.setSpeed(flOutput);
    Robot.chassis.leftBackPod.setSpeed(blOutput);
    Robot.chassis.rightBackPod.setSpeed(brOutput);
    //turn SwervePODs
    Robot.chassis.rightFrontPod.setAngle(frHeading);
    Robot.chassis.leftFrontPod.setAngle(flHeading);
    Robot.chassis.leftBackPod.setAngle(blHeading);
    Robot.chassis.rightBackPod.setAngle(brHeading);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return flFollower.isFinished()&&frFollower.isFinished()&&blFollower.isFinished()&&brFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //call swerveDrive stop
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
