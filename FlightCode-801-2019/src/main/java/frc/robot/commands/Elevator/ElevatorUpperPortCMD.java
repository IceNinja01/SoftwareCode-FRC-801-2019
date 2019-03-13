/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Constants;

public class ElevatorUpperPortCMD extends Command {
  public ElevatorUpperPortCMD() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.updatePID();
    Robot.elevator.updateSmartMotion();
    Robot.elevator.elevatorRun(Constants.ElevatorUpperPosition);
    Robot.elevator.carriageRun(Constants.CarriageUpperPosition);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // updates both elevator and carriage dashboard readings
    Robot.elevator.elevatorEncoderPos();  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.elevator.elevatorIsMoving() 
            && !Robot.elevator.carriageIsMoving();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
