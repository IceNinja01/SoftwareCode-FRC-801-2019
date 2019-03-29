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
	public static ADIS16448_IMU imu = new ADIS16448_IMU();

	
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
	private double kP_d = 0.0001;
	private double kD_d = 0.0;
	private double kI_d = 0.0;
	private double kIz_d = 0.0;
	private double kFF_d = 0.0;
	private double kMaxOutput_d = 1.0;

	private double kP_t = 0.001;
	private double kD_t = 0.0;
	private double kI_t = 0.0;
	private double kIz_t = 0.0;
	private double kFF_t = 0.0;
	private double kMaxOutput_t = 1.0;
	private double maxLimit= 1.0;

	public Chassis(){
		super(0.02, 0.0000001, 0.8, 0.001);
		getPIDController().setAbsoluteTolerance(2.0);
		getPIDController().setInputRange(0.0, 360.0);
		getPIDController().setContinuous(true);
		getPIDController().setOutputRange(-0.3, 0.3);
		enable();
	}

	public void init(){
		rightFrontPod = new SwervePOD(Constants.RightFrontDriveMotorID, Constants.RightFrontSteerMotorID, MotorName.RightFront);
		leftFrontPod = new SwervePOD(Constants.LeftFrontDriveMotorID, Constants.LeftFrontSteerMotorID, MotorName.LeftFront);
		leftBackPod = new SwervePOD(Constants.LeftRearDriveMotorID, Constants.LeftRearSteerMotorID, MotorName.LeftBack);
		rightBackPod = new SwervePOD(Constants.RightRearDriveMotorID, Constants.RightRearSteerMotorID, MotorName.RightBack);

		
		leftBackPod.invertDriveMotor(true);
		rightBackPod.invertDriveMotor(true);

		chassisSwerveDrive = new SwerveDrive(rightFrontPod, leftFrontPod, leftBackPod, rightBackPod, 1);

		//chassisSwerveDrive.configPIDDrive(0.0001, 0.00000, 0.0, 0.0, .0001818, -1.0, 1.0);
		chassisSwerveDrive.configPIDDrive( Constants.kP_DriveMotors, Constants.kI_DriveMotors, 
				Constants.kD_DriveMotors, Constants.kIz_DriveMotors, Constants.kFF_DriveMotors, 
				Constants.kOutputRangeMmin_DriveMotors,Constants.kOutputRangeMax_DriveMotors );


		//chassisSwerveDrive.configPIDTurn(0.005, 0.0001, 0.0000, 0, 0.0000, -1, 1, 0);
		chassisSwerveDrive.configPIDTurn( Constants.kP_TurnMotors, Constants.kI_TurnMotors, 
				Constants.kD_TurnMotors, Constants.kIz_TurnMotors, Constants.kFF_TurnMotors, 
				Constants.kOutputRangeMmin_TurnMotors, Constants.kOutputRangeMax_TurnMotors, 
				Constants.deadBand_TurnMotors );

		chassisSwerveDrive.currentDriveLimit(50, 50);

		chassisSwerveDrive.brakeOff();
		rightFrontPod.setBias();
		leftFrontPod.setBias();
		leftBackPod.setBias();
		rightBackPod.setBias();
		
		// leftFrontPod.setInvertTurn(true);
		initDashboard();
		imu.calibrate();
		imu.reset();
	
	}

	public void initDefaultCommand() {
    	setDefaultCommand(new DriveWithJoysticks());

	}

	public void initDashboard(){
		 // specify widget properties here
		 setPoint_drive = tab_chassis.add("SetPoint_Drive", 0.0).getEntry(); //input is in Inches
		 angle_input = tab_chassis.add("Angle_input", 0).getEntry(); 
		 velocity_input = tab_chassis.add("Velocity_input", 0).getEntry(); 
		//DriveMotors
		ShuffleboardLayout driveMotors = Shuffleboard.getTab(ChassisTitle)
		.getLayout("DriveMotors", BuiltInLayouts.kList)
		.withSize(2, 4)
		.withPosition(0, 0);
		kP_drive = driveMotors.add("kP_drive", kP_d).withPosition(0, 0).getEntry();
		kI_drive = driveMotors.add("kI_drive", kD_d).withPosition(0, 1).getEntry();
		kD_drive = driveMotors.add("kD_drive", kI_d).withPosition(0, 2).getEntry();
		kIz_drive = driveMotors.add("kIz_drive", kIz_d).withPosition(0, 3).getEntry(); 
		kFF_drive = driveMotors.add("kFF_drive", kFF_d).withPosition(0, 4).getEntry(); 
		kMaxOutput_drive = driveMotors.add("kMaxOutput_drive", kMaxOutput_d)
		.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1))
		.withPosition(0, 5).getEntry();
		
		//SteerMotors
		ShuffleboardLayout steerMotors = Shuffleboard.getTab(ChassisTitle)
		 .getLayout("SteerMotors", BuiltInLayouts.kList)
		 .withSize(2, 4)
		 .withPosition(2, 0);
		kP_turn = steerMotors.add("kP_turn", kP_t).withPosition(0, 0).getEntry();
		kD_turn = steerMotors.add("kD_turn", kD_t).withPosition(0, 1).getEntry();
		kI_turn = steerMotors.add("kI_turn", kI_t).withPosition(0, 2).getEntry();
		kIz_turn = steerMotors.add("kIz_turn", kIz_t).withPosition(0, 3).getEntry(); 
		kFF_turn = steerMotors.add("kFF_turn", kFF_t).withPosition(0, 4).getEntry(); 
		kMaxOutput_turn = steerMotors.add("kMaxOutput_turn", kMaxOutput_t)
		.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1))
		.withPosition(0, 5).getEntry(); 

		// kMaxOutput_turn = tab_chassis.add("kMaxOutput_turn", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min_", 0, "max_", 1)).getEntry();
		 // specify widget properties here

		ShuffleboardLayout chassisCommands = Shuffleboard.getTab(ChassisTitle)
			.getLayout("ChassisCommands", BuiltInLayouts.kList)
			.withSize(2, 2)
			.withPosition(4, 0)
			.withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
		chassisCommands.add(new UpdateChassisPID()).withPosition(0, 0);
		chassisCommands.add(new DriveFwd(0, 0, 0)).withPosition(0, 1);

	}
	
	public void motorDrive(double x, double y, double z) {
			
		if(Robot.elevator.elevatorIsUp()){
			chassisSwerveDrive.setMaxRPM(0.5);
		}
		else{
			chassisSwerveDrive.setMaxRPM(0.8);
		}
		x = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getX(),1.5), 0.05, maxLimit);
		y = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getY(),1.5), 0.05, maxLimit);
		z = Utils.limitMagnitude(Utils.joyExpo(Robot.oi.driver.getRawAxis(4),1.5), 0.05, maxLimit/2);
		
		
		if(robotOrient){ //Field oriented
			chassisSwerveDrive.drive(x,y,z,getGyroAngle());
		}
		else{//Robot oriented
			chassisSwerveDrive.drive(x,y,z, 0.0);
		}
	 }

	 public void setPIDDrive(){
		chassisSwerveDrive.configPIDDrive(kP_drive.getDouble(kP_d), kD_drive.getDouble(kD_d), 
			kI_drive.getDouble(kI_d), kIz_drive.getDouble(kIz_d), kFF_drive.getDouble(kFF_d), 
			-kMaxOutput_drive.getDouble(kMaxOutput_d), kMaxOutput_drive.getDouble(kMaxOutput_d));
	 }

	 public void setPIDTurn(){
		chassisSwerveDrive.configPIDTurn(kP_turn.getDouble(0.001), kD_turn.getDouble(0.00001), kI_turn.getDouble(0.0001), 0, 0.00001,
		 -kMaxOutput_turn.getDouble(1.0), kMaxOutput_turn.getDouble(1.0) ,3);
	 }

	 public void motorDrive_CMD(double angle, double velocity){
		 chassisSwerveDrive.turnMotorsRPM(angle, velocity);
	 }

	public void stop() {   
	    	chassisSwerveDrive.stopMotor();
		}

	public void toggleFieldOrient() {
		robotOrient = true;
	}
	
	public void toggleRobotOrient() {
		if(robotOrient){
			robotOrient = false;
		} else {
			robotOrient = true;
		}
	}

	public void getDriveVoltage(){
		chassisSwerveDrive.getDriveVoltage();
	}

	public void getTurnVoltage(){
		chassisSwerveDrive.getTurnVoltage();
	}

	public boolean isDistance(double setPoint){
		return chassisSwerveDrive.isDistance(setPoint_drive.getDouble(0.0));
	}

	public void getUpdateSpeed(){
		chassisSwerveDrive.getUpdate();
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

	public void updateSD() {

		  SmartDashboard.putNumber("GyroAngle", getGyroAngle());
	}

}

