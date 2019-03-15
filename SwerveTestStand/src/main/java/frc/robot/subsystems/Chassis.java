package frc.robot.subsystems;

import frc.robot.Constants;
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
		rightFrontPod = new SwervePOD(Constants.RightFrontDrive, Constants.RightFrontSteer, MotorName.RightFront);
		rightFrontPod.configPIDDrive(0.0002, 0.0, 0.0, 0.0, 0.1, -1.0, 1.0);

		rightFrontPod.configPIDTurn(0.005, 0.00000, 0.0000, 0, 0.0001, -0.5, 0.5, 2);
		// rightFrontPod.brakeOff();
		// rightFrontPod.invertDriveMotor(true);
		chassisSwerveDrive = new SwerveDriveTwoMotors(rightFrontPod, 5);
		chassisSwerveDrive.setDriveCurrentLimit(10, 40);
	}

	public void initDefaultCommand() {
    	setDefaultCommand(new DriveWithJoysticks());

	}
	
	public void motorDrive(double x, double y, double z, double angleCMD) {
	    	
	        x = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getX(),1.5), 0.1, 1.0);
	        y = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getY(),1.5), 0.1, 1.0);
	        z = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getRawAxis(4),1.5), 0.05, 1.0);
	        
	        chassisSwerveDrive.drive(x,y,z,0.0);
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

	public void setAngle(double angleCMD) {
		chassisSwerveDrive.turnMotors(angleCMD);
	}

	public boolean isTarget() {
		return rightFrontPod.onTarget();
	}
		
}

