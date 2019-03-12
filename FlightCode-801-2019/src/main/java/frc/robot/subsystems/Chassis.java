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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
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

		chassisSwerveDrive.configPIDTurn(0.01, 0.01, 0.00001, 0, 0.0001, -1.0, 1.0, 2);
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
		 // specify widget properties here
		 setPoint_drive = tab_chassis.add("SetPoint_Drive", 0.0).getEntry(); //input is in Inches
		//DriveMotors
		ShuffleboardLayout driveMotors = Shuffleboard.getTab(ChassisTitle)
		.getLayout("DriveMotors", BuiltInLayouts.kList)
		.withSize(2, 2)
		.withPosition(2, 0);
		kP_drive = driveMotors.add("kP_drive", .0005).getEntry();
		kI_drive = driveMotors.add("kI_drive", 1e-6).getEntry();
		kD_drive = driveMotors.add("kD_drive", 0).getEntry();
		kIz_drive = driveMotors.add("kIz_drive", 0).getEntry(); 
		kFF_drive = driveMotors.add("kFF_drive", 0).getEntry(); 
		kMaxOutput_drive = driveMotors.add("kMaxOutput_drive", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
		
		//SteerMotors
		ShuffleboardLayout steerMotors = Shuffleboard.getTab(ChassisTitle)
		 .getLayout("SteerMotors", BuiltInLayouts.kList)
		 .withSize(2, 2)
		 .withPosition(4, 0);
		kP_turn = steerMotors.add("kP_turn", 0.01).getEntry();
		kI_turn = steerMotors.add("kI_turn", 0.01).getEntry();
		kD_turn = steerMotors.add("kD_turn", 0).getEntry();
		kIz_turn = steerMotors.add("kIz_turn", 0).getEntry(); 
		kFF_turn = steerMotors.add("kFF_turn", 0.0001).getEntry(); 
		kMaxOutput_turn = steerMotors.add("kMaxOutput_turn", 1.0).getEntry(); 

		// kMaxOutput_turn = tab_chassis.add("kMaxOutput_turn", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min_", 0, "max_", 1)).getEntry();
		 // specify widget properties here
		 angle_input = tab_chassis.add("Angle_input", 0).getEntry(); 
		 velocity_input = tab_chassis.add("Velocity_input", 0).getEntry(); 

		ShuffleboardLayout chassisCommands = Shuffleboard.getTab(ChassisTitle)
			.getLayout("ChassisCommands", BuiltInLayouts.kList)
			.withSize(2, 2)
			.withPosition(0, 0)
			.withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
		chassisCommands.add(new UpdateChassisPID());
		chassisCommands.add(new DriveFwd(0, 0, 0));

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
		chassisSwerveDrive.configPIDDrive(kP_drive.getDouble(0.0005), kD_drive.getDouble(0.0001), kI_drive.getDouble(0.0001), 0.0, 0.0, 
		-kMaxOutput_drive.getDouble(1.0), kMaxOutput_drive.getDouble(1.0));
	 }

	 public void setPIDTurn(){
		chassisSwerveDrive.configPIDTurn(kP_turn.getDouble(0.01), kD_turn.getDouble(0.0001), kI_turn.getDouble(0.001), 0, 0.0001,
		 -kMaxOutput_turn.getDouble(1.0), kMaxOutput_turn.getDouble(1.0) ,3);
	 }

	 public void motorDrive_CMD(double angle, double velocity){
		 chassisSwerveDrive.turnMotorsRPM(angle_input.getDouble(0.0), velocity_input.getDouble(0.0));
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

