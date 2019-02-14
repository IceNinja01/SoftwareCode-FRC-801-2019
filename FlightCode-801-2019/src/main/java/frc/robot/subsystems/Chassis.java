package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Utilities.Utils;
import frc.robot.commands.DriveWithJoysticks;

import frc.robot.SwerveClass.SwerveDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveClass.SwervePOD;
import frc.robot.SwerveClass.Team801TalonSRX;
import frc.robot.SwerveClass.SwervePOD.MotorName;

/**
 *
 */
public class Chassis extends Subsystem {

	public static SwervePOD rightFrontPod;
	public static SwervePOD leftFrontPod;
	public static SwervePOD leftBackPod;
	public static SwervePOD rightBackPod;

	
	public static SwerveDrive chassisSwerveDrive;

	public void init(){
		rightFrontPod = new SwervePOD(0, 4, MotorName.RightFront);
		leftFrontPod = new SwervePOD(0, 4, MotorName.LeftFront);
		leftBackPod = new SwervePOD(0, 4, MotorName.LeftBack);
		rightBackPod = new SwervePOD(0, 4, MotorName.RightBack);

		chassisSwerveDrive = new SwerveDrive(rightFrontPod, leftFrontPod, leftBackPod, rightBackPod, 3);

		chassisSwerveDrive.configPIDDrive(0.00001, 0.000001, 0.0, 0.0, 0.0, -1.0, 1.0);

		chassisSwerveDrive.configPIDTurn(0.0025, 0.001, 0.001, 0, 0.001, -1.0, 1.0, 1);

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
		
}

