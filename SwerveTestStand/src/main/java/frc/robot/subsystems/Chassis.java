package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Utilities.Utils;
import frc.robot.commands.DriveWithJoysticks;

import frc.robot.SwerveClass.SwerveDriveTwoMotors;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveClass.SwervePOD;
import frc.robot.SwerveClass.Team801TalonSRX;

/**
 *
 */
public class Chassis extends Subsystem {

	private static SwervePOD rightFrontPod;
	
	private static SwerveDriveTwoMotors chassisSwerveDrive;

	public void init(){
		rightFrontPod = new SwervePOD(0, 4, 0);
		rightFrontPod.configPIDDrive(0.00001, 0.000001, 0.0, 0.0, 0.0, -1.0, 1.0);
		rightFrontPod.configPIDTurn(0.005, 0.0, 0.005, 0, 0.0, -0.25, 0.25, 2.0);
		chassisSwerveDrive = new SwerveDriveTwoMotors(rightFrontPod, 1);
	}

	public void initDefaultCommand() {
    	setDefaultCommand(new DriveWithJoysticks());

	}
	
	public void motorDrive(double x, double y, double z, double angleCMD) {
	    	
	        x = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getX(),1.5), 0.1, 1.0);
	        y = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getY(),1.5), 0.1, 1.0);
	        z = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getRawAxis(4),1.5), 0.1, 1.0);
	        

	        chassisSwerveDrive.drive(x,y,z,angleCMD);

	 }

	public void stop() {
	   
	    	chassisSwerveDrive.stopMotor();
	    }
		
}

