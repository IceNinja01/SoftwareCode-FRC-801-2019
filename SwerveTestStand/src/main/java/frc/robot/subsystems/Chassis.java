package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.Utilities.Utils;
import frc.robot.commands.DriveWithJoysticks;

import frc.robot.SwerveClass.SwerveDriveTwoMotors;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveClass.SwervePOD;
import frc.robot.SwerveClass.SwervePOD.MotorName;

/**
 *
 */
public class Chassis extends Subsystem {

	private static SwervePOD rightFrontPod;
	
	public static SwerveDriveTwoMotors chassisSwerveDrive;

	public void init(){
		rightFrontPod = new SwervePOD(1, 4, MotorName.RightFront);
		rightFrontPod.configPIDDrive(0.00005, 0.000001, 0.0, 0.0, 0.0, -1.0, 1.0);

		rightFrontPod.configPIDTurn(0.0015, 0.00000, 0.0000, 1, 0.0001, -0.5, 0.5, 1);
		rightFrontPod.brakeOff();
		
		chassisSwerveDrive = new SwerveDriveTwoMotors(rightFrontPod, 1);
		chassisSwerveDrive.setDriveCurrentLimit(10, 40);
	}

	public void initDefaultCommand() {
    	setDefaultCommand(new DriveWithJoysticks());

	}
	
	public void motorDrive(double x, double y, double z, double angleCMD) {
	    	
	        x = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getX(),1.5), 0.05, 1.0);
	        y = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getY(),1.5), 0.05, 1.0);
	        z = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getRawAxis(4),1.5), 0.05, 1.0);
	        
	        chassisSwerveDrive.drive(x,y,z,angleCMD);
	 }

	public void stop() {
	   
	    	chassisSwerveDrive.stopMotor();
		}
		
	public void getAngle(){
		rightFrontPod.getAngleDeg();
		rightFrontPod.getAbsAngle();
	}

	public void getError() {
		rightFrontPod.getTurnPIDError();

	}
		
}

