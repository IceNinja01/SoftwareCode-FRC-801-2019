package org.usfirst.frc.team801.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team801.robot.Robot;
import org.usfirst.frc.team801.robot.RobotMap;
import org.usfirst.frc.team801.robot.Utilities.Utils;
import org.usfirst.frc.team801.robot.commands.DriveWithJoysticks;

import SwerveClass.SwerveDriveTwoMotors;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveClass.SwervePOD;

/**
 *
 */
public class Chassis extends Subsystem {
	private static CANSparkMax rightFrontDrive = frc.robot.RobotMap.rightFrontDrive;
	private static Team801TalonSRX rightFrontTurn = frc.robot.RobotMap.rightFrontTurn;
	private static SwervePOD rightFrontPod = new SwervePOD(Drive, frc.robot.RobotMap., 0)
	private static SwerveDriveTwoMotors chassisSwerveDrive = RobotMap.swerveDrive;

	public void initDefaultCommand() {
    	setDefaultCommand(new DriveWithJoysticks());

	}
	
	 public void motorDrive(double x, double y, double z, double angleCMD) {
	    	
	        x = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getX(),1.5), 0, 0.7);
	        y = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getY(),1.5), 0, 0.7);
	        z = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getRawAxis(4),1.5), 0, 1.0);
	        

	        chassisSwerveDrive.drive(x,y,z,angleCMD);

	 }

	public void stop() {
	   
	    	chassisSwerveDrive.stopMotor();
	    }
		
}

