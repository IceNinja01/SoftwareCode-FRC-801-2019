package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utilities.ADIS16448_IMU;
import frc.robot.Utilities.Utils;
import frc.robot.commands.DriveWithJoysticks;

import frc.robot.SwerveClass.SwerveDrive;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveClass.SwervePOD;
import frc.robot.SwerveClass.SwervePOD.MotorName;

/**
 *
 */
public class Chassis extends PIDSubsystem {

	public static SwervePOD rightFrontPod;
	public static SwervePOD leftFrontPod;
	public static SwervePOD leftBackPod;
	public static SwervePOD rightBackPod;
	public static ADIS16448_IMU imu;

	
	public static SwerveDrive chassisSwerveDrive;
	private double angle = 0;
	private double biasAngle = 0;
	private double headingError;
	private double zRateCmd;
	private double headingCMD;
	private boolean robotOrient = true;

	public Chassis(){
		super(0.02, 0.0000001, 0.8, 0.001);
		getPIDController().setAbsoluteTolerance(2.0);
		getPIDController().setInputRange(0.0, 360.0);
		getPIDController().setContinuous(true);
		getPIDController().setOutputRange(-0.3, 0.3);
		enable();
	}

	public void init(){
		rightFrontPod = new SwervePOD(Constants.RightFrontDrive, Constants.RightFrontSteer, MotorName.RightFront);
		leftFrontPod = new SwervePOD(Constants.LeftFrontDrive, Constants.LeftFrontSteer, MotorName.LeftFront);
		leftBackPod = new SwervePOD(Constants.LeftRearDrive, Constants.LeftRearSteer, MotorName.LeftBack);
		rightBackPod = new SwervePOD(Constants.RightRearDrive, Constants.RightRearSteer, MotorName.RightBack);

		chassisSwerveDrive = new SwerveDrive(rightFrontPod, leftFrontPod, leftBackPod, rightBackPod, 3);

		chassisSwerveDrive.configPIDDrive(0.00001, 0.000001, 0.0, 0.0, 0.0, -1.0, 1.0);

		chassisSwerveDrive.configPIDTurn(0.0025, 0.001, 0.001, 0, 0.001, -1.0, 1.0, 1);

		chassisSwerveDrive.setDriveCurrentLimit(20, 40);

		chassisSwerveDrive.brakeOff();
	
	//	imu.calibrate();
	//	imu.reset();
	
	}

	public void initDefaultCommand() {
    	setDefaultCommand(new DriveWithJoysticks());

	}
	
	public void motorDrive(double x, double y, double z) {
	    	
		x = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getX(),1.5), 0.05, 1.0);
		y = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getY(),1.5), 0.05, 1.0);
		z = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getRawAxis(4),1.5), 0.05, 1.0);
		
		if(robotOrient){ //Field oriented
			chassisSwerveDrive.drive(x,y,z,getGyroAngle());
		}
		else{//Robot oriented
			chassisSwerveDrive.drive(x,y,z,0.0);
		}

	 }

	public void stop() {   
	    	chassisSwerveDrive.stopMotor();
		}

	public void toggleFieldOrient() {
		robotOrient = true;
	}
	
	public void toggleRobotOrient() {
		robotOrient = false;
	}

	//===============================================================================//
	//Imu code
	public double getGyroAngle() {
		angle = Utils.wrapAngle0To360Deg(imu.getAngleZ() - biasAngle );
		
		return angle;
	}
	
	public double getAngleX() {
		double x = imu.getAngleX();
		return x;
	}
	public double getAngleY() {
		double y = imu.getAngleY();
		return y;
	}

	public void setGyroBias() {
		biasAngle = Utils.wrapAngle0To360Deg(imu.getAngleZ());
	}
	
	protected double returnPIDInput() {
		return headingError;
	}
	protected void usePIDOutput(double output) {
		zRateCmd = output;
	}

	public double getHeadingErr() {
		return headingError;
	}

	public double getZRateCmd() {
		return zRateCmd;
	}

	public double getHeadingCmd() {
		return headingCMD;
	}
}

