package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class ToggleFieldDrive extends Command {

    public ToggleFieldDrive() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//    	Robot.chassis.toggleRobotOrient();
    
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.chassis.toggleRobotOrient();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
   	  Robot.chassis.toggleFieldOrient();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}