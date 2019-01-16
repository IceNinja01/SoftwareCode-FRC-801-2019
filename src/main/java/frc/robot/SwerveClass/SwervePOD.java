package frc.robot.SwerveClass;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;

public class SwervePOD {
	
	private static final String kHallSensor = null;
	private CANSparkMax driveMotor;
	private CANSparkMax turnMotor;
	private PIDSource pidTurnSource;
	private PIDController pidTurnController;
	private MotorName motorName;
	private CANEncoder driveMotorEnc;
	private CANEncoder turnMotorEnc;
	private CANPIDController drivePID;
	private CANPIDController turnPID;
	private double k_drive_P, k_drive_I, k_drive_D, k_drive_Iz, k_drive_FF, kMinRPM, kMaxRPM;
	private double k_turn_P, k_turn_I, k_turn_D, k_turn_Iz, k_turn_FF, kMinAngle, kMaxAngle;
	/**
	 * 
	 * @param Drive Motor Number on PDB for the Drive motor on SwervePOD
	 * @param Turn	Motor Number on PDB for the Turn motor on SwervePOD
	 * @param PODName EnumType for POD Name: RightFront(0), LeftFront(1), LeftBack(2), RightBack(3).
	 */
	
	public SwervePOD(int Drive, int Turn, int PODName) {
//		Initialize motors
		driveMotor  = new CANSparkMax(Drive, CANSparkMaxLowLevel.MotorType.kBrushless);
		driveMotorEnc = driveMotor.getEncoder();
		drivePID = driveMotor.getPIDController();

		turnMotor  = new CANSparkMax(Turn, CANSparkMaxLowLevel.MotorType.kBrushless);
		turnMotorEnc = turnMotor.getEncoder();
		turnPID = turnMotor.getPIDController();
		motorName.value = PODName;
	}
	
		
	public enum MotorName{
		RightFront(0),
		LeftFront(1),
		LeftBack(2),
		RightBack(3);
		public int value;
		MotorName(int initValue){
			
			this.value = initValue;
			
		}		
	}
	
	public void initialize() {
		

	
	/*the sensor and motor must be
	in-phase. This means that the sensor position must move in a positive direction as the motor
	controller drives positive motor output. To test this, first drive the motor manually (using
	gamepad axis for example). Watch the sensor position either in the roboRIO Web-based
	Configuration Self-Test, or by calling GetSelectedSensorPosition() and printing it to console.
	If the Sensor Position moves in a negative direction while Talon SRX motor output is positive
	(blinking green), then use the setSensorPhase() routine/VI to multiply the sensor position by (-
	1). Then retest to confirm Sensor Position moves in a positive direction with positive motor
	drive.**/
	turnMotor.setSensorPhase(false); 


	driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	/* set the peak and nominal outputs, 12V means full */
	driveMotor.configNominalOutputForward(0, 10);
	driveMotor.configNominalOutputReverse(0, 10);
	driveMotor.configPeakOutputForward(11.0, 10);
	driveMotor.configPeakOutputReverse(-11.0, 10);
	driveMotor.enableVoltageCompensation(true); 
	driveMotor.configNeutralDeadband(0.01, 10);
	driveMotor.configAllowableClosedloopError(0, 0, 10);		
	/* Set the motors PIDF constants**/
	//index 0
	driveMotor.config_kF(0, .026, 10);
	driveMotor.config_kP(0, .051, 10);
	driveMotor.config_kI(0, 0.0, 10);
	driveMotor.config_kD(0, 0.5, 10);
	driveMotor.setSelectedSensorPosition(0, 0, 10);
	}
	
	public void configPIDTurn(double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput) {
	    // set PID coefficients for turn motor
		turnPID.setP(kP);
		turnPID.setI(kI);
		turnPID.setD(kD);
		turnPID.setIZone(kIz);
		turnPID.setFF(kFF);
		turnPID.setOutputRange(kMinOutput, kMaxOutput);
					
	}

	public void configTurnEnconder(){
		turnMotor.setParameter(ConfigParameter.kMotorL, kMotorL );
	// pidTurnController = new PIDController(kP, kI, kD, pidTurnSource, turnMotor);
	// pidTurnController.setAbsoluteTolerance(deadBand);
	// pidTurnController.setInputRange(0, 360);
	// pidTurnController.setContinuous(true);
	// pidTurnController.setOutputRange(-maxTurnVoltage, maxTurnVoltage);
	// pidTurnController.enable();
	
	}
	
	public double getAngle() {
		   int motorNumber = turnMotor.getDeviceID();
		   // Convert timeUs Pulse to angle
		   /* get the period in us, rise-to-rise in microseconds */
		   double timeUs = turnMotor.getSensorCollection().getPulseWidthRiseToRiseUs();
		   // Convert timeUs Pulse to angle	   
		   double degrees = turnMotor.getSensorCollection().getPulseWidthRiseToFallUs()*(360.0/timeUs);
		   SmartDashboard.putNumber("RawAngle_"+motorName.name(), degrees);
		   degrees = Utils.wrapAngle0To360Deg(degrees) - Constants.AngleBias[motorName.value];
		   degrees = Utils.wrapAngle0To360Deg(degrees);
		   SmartDashboard.putNumber(motorName.name()+" turn", degrees);
		   return degrees;
	}
	
	public void setSpeed(double speed) {
		drivePID.setReference(speed*4800*4096/600 , ControlType.kVelocity);
	}
	
	public void turn(double angle) {
		pidTurnController.setSetpoint(angle);
	}
	
	public void stop() {
		drivePID.setReference(0 , ControlType.kVelocity);
		driveMotor.stopMotor();
		pidTurnController.disable();
		turnMotor.set(ControlMode.PercentOutput, 0.0);
	}
	
	public void brakeOn() {
		driveMotor.setIdleMode(IdleMode.kBrake);
	  	pidTurnController.disable();

	}
	
	public void brakeOff() {
		driveMotor.setIdleMode(IdleMode.kCoast);
		drivePID.setReference(0 , ControlType.kVelocity);
	  	pidTurnController.enable();
	}
	
	public double getAmps() {
		return driveMotor.getOutputCurrent();
	}
	
	public double getVoltage() {
		return driveMotor.getMotorOutputVoltage();
	}
	
	public void setDriveCurrentLimit(int peakAmps, int durationMs, int continousAmps) {
		driveMotor.configPeakCurrentLimit(peakAmps, 10); /* 35 A */
		driveMotor.configPeakCurrentDuration(durationMs, 10); /* 200ms */
		driveMotor.configContinuousCurrentLimit(continousAmps, 10); /* 30A */
		driveMotor.enableCurrentLimit(true); /* turn it on */
	}

	public double getSpeed(){
		//Add the following constants to make proper speed calculations
		//(kMaxRPM  / 600) * (kSensorUnitsPerRotation / kGearRatio)
		return driveMotorEnc.getVelocity(); 
	}

	public int getPosition(){
		return (int) driveMotorEnc.getPosition();
	}
}
