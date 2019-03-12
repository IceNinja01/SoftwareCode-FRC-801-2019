package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utilities.Utils;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.Chassis.DriveFwd;
import frc.robot.commands.Chassis.UpdateChassisPID;
import frc.robot.SwerveClass.SwerveDrive;

import java.util.Map;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveClass.SwervePOD;
import frc.robot.SwerveClass.SwervePOD.MotorName;

/**
 *
 */
public class Chassis extends PIDSubsystem {

	private static final String ChassisTitle = "Chassis";
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
	private ShuffleboardTab tab_chassis = Shuffleboard.getTab(ChassisTitle);
	private NetworkTableEntry setPoint_drive;
	private NetworkTableEntry kP_drive;
	private NetworkTableEntry kI_drive;
	private NetworkTableEntry kD_drive;
	private NetworkTableEntry kIz_drive;
	private NetworkTableEntry kFF_drive;
	private NetworkTableEntry kMaxOutput_drive;
	private NetworkTableEntry kP_turn;
	private NetworkTableEntry kI_turn;
	private NetworkTableEntry kD_turn;
	private NetworkTableEntry kIz_turn;
	private NetworkTableEntry kFF_turn;
	private NetworkTableEntry kMaxOutput_turn;
	private NetworkTableEntry angle_input;
	private NetworkTableEntry velocity_input;


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

		
		leftBackPod.invertDriveMotor(true);
		rightBackPod.invertDriveMotor(true);
		chassisSwerveDrive = new SwerveDrive(rightFrontPod, leftFrontPod, leftBackPod, rightBackPod, 3);

		chassisSwerveDrive.configPIDDrive(0.0002, 0.00000, 0.0, 0.0, 0.0, -1.0, 1.0);

		chassisSwerveDrive.configPIDTurn(0.000015, 0.0001, 0.001, 0, 0.0001, -0.15, 0.15, 1);
		// leftBackPod.configPIDTurn(0.00015,  0.0001, 0.001, 0, 0.001, -0.15, 0.15, 1);
		
		chassisSwerveDrive.setDriveCurrentLimit(20, 40);

		chassisSwerveDrive.brakeOff();
		rightFrontPod.setBias();
		leftFrontPod.setBias();
		leftBackPod.setBias();
		rightBackPod.setBias();
		
		leftFrontPod.setInvertTurn(true);
		initDashboard();
		// imu.calibrate();
		// imu.reset();
	
	}

	public void initDefaultCommand() {
    	setDefaultCommand(new DriveWithJoysticks());

	}

	public void initDashboard(){
		setPoint_drive = tab_chassis.add("SetPoint_Drive", 0.0).getEntry(); //input is in Inches
		kP_drive = tab_chassis.add("kP_drive", .0005).getEntry();
		kI_drive = tab_chassis.add("kI_drive", 1e-6).getEntry();
		kD_drive = tab_chassis.add("kD_drive", 0).getEntry();
		kIz_drive = tab_chassis.add("kIz_drive", 0).getEntry(); 
		kFF_drive = tab_chassis.add("kFF_drive", 0).getEntry(); 
		kMaxOutput_drive = tab_chassis.add("kMaxOutput_drive", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
		 // specify widget properties here
		kP_turn = tab_chassis.add("kP_turn", .0005).getEntry();
		kI_turn = tab_chassis.add("kI_turn", 1e-6).getEntry();
		kD_turn = tab_chassis.add("kD_turn", 0).getEntry();
		kIz_turn = tab_chassis.add("kIz_turn", 0).getEntry(); 
		kFF_turn = tab_chassis.add("kFF_turn", 0).getEntry(); 
		kMaxOutput_turn = tab_chassis.add("kMaxOutput_turn", 0.15).getEntry(); 

		// kMaxOutput_turn = tab_chassis.add("kMaxOutput_turn", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min_", 0, "max_", 1)).getEntry();
		 // specify widget properties here
		 angle_input = tab_chassis.add("Angle_input", 0).getEntry(); 
		 velocity_input = tab_chassis.add("Velocity_input", 0).getEntry(); 

	
		tab_chassis.add(new UpdateChassisPID()).withPosition(2, 2).withSize(2, 1);

	}
	
	public void motorDrive(double x, double y, double z) {
	    	
		x = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getX(),1.5), 0.05, 1.0);
		y = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getY(),1.5), 0.05, 1.0);
		z = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getRawAxis(4),1.5), 0.05, 1.0);
		
		if(robotOrient){ //Field oriented
			chassisSwerveDrive.drive(x,y,z,0.0);
		}
		else{//Robot oriented
			chassisSwerveDrive.drive(x,y,z,0.0);
		}

	 }

	 public void setPIDDrive(){
		chassisSwerveDrive.configPIDDrive(kP_drive.getDouble(0.0005), kD_drive.getDouble(0.0005), 0.0, 0.0, 0.0, 
		-kMaxOutput_drive.getDouble(1.0), kMaxOutput_drive.getDouble(1.0));
	 }

	 public void setPIDTurn(){
		chassisSwerveDrive.configPIDTurn(kP_turn.getDouble(0.00001), 0.0001, 0.001, 0, 0.0001,
		 -kMaxOutput_turn.getDouble(0.01), kMaxOutput_turn.getDouble(0.01) ,3);
	 }

	 public void motorDrive_CMD(double angle, double velocity){
		 chassisSwerveDrive.turnMotorsRPM(angle_input.getDouble(0.0), 100);
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

	public void getDriveVoltage(){
		chassisSwerveDrive.getDriveVoltage();
	}

	public void getTurnVoltage(){
		chassisSwerveDrive.getTurnVoltage();
	}

	public boolean isDistance(double setPoint){
		return chassisSwerveDrive.isDistance(setPoint_drive.getDouble(10.0));
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

