package frc.robot.SwerveClass;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

//import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.wpilibj.PIDSource;
//import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Utilities.PID;
//import frc.robot.Utilities.Utils;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;

public class SwervePOD {

	//**********************************
  	// Class private variables
  	//**********************************

	private MotorName motorName;

	private CANSparkMax driveMotor;
	private CANEncoder driveMotorEnc;
	private CANPIDController drivePID;

	private TalonSRX turnMotor;
	private PID turnMotorPID;

	//private double k_drive_P, k_drive_I, k_drive_D, k_drive_Iz, k_drive_FF, kMinRPM, kMaxRPM;
	//private double k_turn_P, k_turn_I, k_turn_D, k_turn_Iz, k_turn_FF, kMinAngle, kMaxAngle;
	//private double angle;

	//private int m_inputRange = 160;

	private int nativeUnits;

	private double last_error;

	//private double turn_kP;

	//private double turn_kD;

	//private double turn_kI;

	private int m_value;

	private boolean kMotorInvert = false;
	private boolean kSensorPhase = true;

	//**********************************
  	// Constructor functions
  	//**********************************
	/**
	 * 
	 * @param Drive Motor Number on PDB for the Drive motor on SwervePOD
	 * @param Turn	Motor Number on PDB for the Turn motor on SwervePOD
	 * @param i EnumType for POD Name: RightFront(0), LeftFront(1), LeftBack(2), RightBack(1).
	 */

	public SwervePOD(int Drive, int Turn, MotorName motorName) {
//		Initialize motors
		driveMotor  = new CANSparkMax(Drive, CANSparkMaxLowLevel.MotorType.kBrushless);
		driveMotorEnc = driveMotor.getEncoder();
		drivePID = driveMotor.getPIDController();

		turnMotor  = new TalonSRX(Turn);	
		this.motorName = motorName;
	}
	
	public enum MotorName{

		RightFront,
		LeftFront,
		LeftBack,
		RightBack
	}
	
	public void initialize() {

	}

	/**
	 * Pos and velocity close loops are calc’d as:
	 *	err = target - posOrVel
	 *	iErr += err
	 *	if IZone != 0 and abs(err) > IZone:
     *		ClearIaccum()
	 *	output = P * err + I * iErr + D * dErr + F * target
	 *	dErr = err - lastErr
	 *	
	 * @param kP proportional constant for PID control
	 * @param kI intergral constant for PID control
	 * @param kD derivative constant for PID control
	 * @param kIz I Zone is specified in the same units as sensor position (ADC units or quadrature edges). If pos/vel error is outside of this value, the integrated error will auto-clear:
	 *				if IZone != 0 and abs(err) > IZone:ClearIaccum()
	 * @param kFF feedforward constant for PID control
	 */
	public void configPIDTurn(double kP, double kI, double kD, int kIz, double kFF, double kMinOutput, double kMaxOutput, int deadBand) {
	    // set PID coefficients for turn motor
		turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		turnMotor.setSensorPhase(kSensorPhase); 
		m_value = deadBand;
		turnMotorPID = new PID(kP, kI, kD, kFF);
		turnMotorPID.setOutputLimits(kMaxOutput);
		turnMotorPID.setContinous(true);
		turnMotorPID.setContinousInputRange(360);

		// /* set the peak and nominal outputs, 12V means full */
		turnMotor.configNominalOutputForward(0, 10);
		turnMotor.configNominalOutputReverse(0, 10);
		turnMotor.configPeakOutputForward(11.0 * kMaxOutput, 10);
		turnMotor.configPeakOutputReverse(-11.0 * kMinOutput, 10);
		turnMotor.enableVoltageCompensation(true); 
		// /* 0.001 represents 0.1% - default value is 0.04 or 4% */
		turnMotor.configNeutralDeadband(0.001, 10);
		// /**
		//  * Grab the 360 degree position of the MagEncoder's absolute
		//  * position, and intitally set the relative sensor to match.
		//  */
		int absolutePosition = getAbsAngle();	

		if (absolutePosition > Constants.AngleBias[0])
		{
			turnMotor.setSelectedSensorPosition(24576- (absolutePosition-Constants.AngleBias[motorName.ordinal()]), 0, 10);
		}
		else
		{
			turnMotor.setSelectedSensorPosition(Constants.AngleBias[motorName.ordinal()]-absolutePosition, 0, 10);
		}
		
		// //set coast mode
		turnMotor.setNeutralMode(NeutralMode.Coast);
		turnMotor.setInverted(kMotorInvert);
		/* 0.001 represents 0.1% - default value is 0.04 or 4% */
		turnMotor.configNeutralDeadband(0.001, 10);
//		//set Voltage for turn motors
		turnMotor.set(ControlMode.PercentOutput, 0.0);
	}
	/**
	 * @param kP proportional constant for PID control
	 * @param kI intergral constant for PID control
	 * @param kD derivative constant for PID control
	 * @param kIz I Zone is specified in the same units as sensor position (ADC units or quadrature edges). If pos/vel error is outside of this value, the integrated error will auto-clear:
	 *				if IZone != 0 and abs(err) > IZone:ClearIaccum()
	 * @param kFF feedforward constant for PID control
	 * @param kMinOutput the minimum percentage to write to the output, -1 min
	 * @param kMaxOutput the maximum percentage to write to the output, 1 max
	 */
	public void configPIDDrive(double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput) {
	    // set PID coefficients for turn motor
		drivePID.setP(kP);
		drivePID.setI(kI);
		drivePID.setD(kD);
		drivePID.setIZone(kIz);
		drivePID.setFF(kFF);
		drivePID.setOutputRange(kMinOutput, kMaxOutput);
		driveMotor.setInverted(true);	
	}

    // reads the actual encoder count
	public int getNativeUnits(){
		int nativeUnits_temp = turnMotor.getSelectedSensorPosition(0);
		SmartDashboard.putNumber("ActualEnc cnt " + motorName, nativeUnits_temp);
		return nativeUnits_temp;
	}

	public double getAngleDeg() {
    	// Convert rotations to degrees	   
		nativeUnits = wrapUnits(getNativeUnits());
		SmartDashboard.putNumber("RelativeEnc " + motorName, nativeUnits);
		double degrees = toDeg(nativeUnits);
		SmartDashboard.putNumber("AngleEncoder "+ motorName, degrees);
		return degrees;
	}

	public int getAbsAngle(){
		int absolutePosition = turnMotor.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		// if (kSensorPhase) { absolutePosition *= -1; }
		// if (kMotorInvert) { absolutePosition *= -1; }
		SmartDashboard.putNumber("AbsoluteEnc " + motorName, absolutePosition);	
		return absolutePosition;
	}
	
	public void setSpeed(double speed) {
		SmartDashboard.putNumber("TargetSpeed " + motorName, speed);	
		drivePID.setReference(speed, ControlType.kVelocity);
	}
	
	public void setAngle(double angle) {
		// Set new position of motor
		turnMotor.set(ControlMode.PercentOutput, turnMotorPID.getOutput(getAngleDeg(), angle));
	}

	public void setDriveEncoder(int counts_per_rev) {
		driveMotor.setParameter(CANSparkMaxLowLevel.ConfigParameter.kEncoderCountsPerRev, counts_per_rev);
	}

	/**
	 * @param stallLimit The current limit in Amps at 0 RPM.
	 * @param freeLimit The current limit at free speed (5700RPM for NEO).
	 */
	public void setDriveCurrentLimit(int stallLimit, int freeLimit) {
		driveMotor.setSmartCurrentLimit(stallLimit, freeLimit);
	}
	/**
	 * @param stallLimit The current limit in Amps at 0 RPM.
	 * @param timeMsec The current limit at free speed (5700RPM for NEO).
	 */
	public void setTurnCurrentLimit(int peakAmps, int durationMs, int continousAmps) {
		turnMotor.configPeakCurrentLimit(peakAmps, 10); /* 15 A */
		turnMotor.configPeakCurrentDuration(durationMs, 10); /* 200ms */
		turnMotor.configContinuousCurrentLimit(continousAmps, 10); /* 10A */
		turnMotor.enableCurrentLimit(true);
	}

	public double getDriveAmps() {
		return driveMotor.getOutputCurrent();
	}
	
	public double getDriveVoltage() {
		return driveMotor.getBusVoltage();
	}

	public double getTurnVolatge(){
		return turnMotor.getMotorOutputVoltage();
	}

	public double getTurnAmps(){
		return turnMotor.getOutputCurrent();
	}

	public double getSpeed(){
		//Add the following constants to make proper speed calculations
		//(kMaxRPM  / 600) * (kSensorUnitsPerRotation / kGearRatio)
		double speed = driveMotorEnc.getVelocity();
		speed = (speed/5.1)*(4*Math.PI)*(1/60.0)*(1/12.0);
		SmartDashboard.putNumber("Motor Speed", speed);
		return speed;
	}

	public int getPosition(){
		return (int) driveMotorEnc.getPosition();
	}

	public void stop() {
		drivePID.setReference(0 , ControlType.kVelocity);
		turnMotor.set(ControlMode.PercentOutput, 0.0);
	}
	
	public void brakeOn() {
		driveMotor.setIdleMode(IdleMode.kBrake);
		// pidTurnController.disable();
		turnMotor.setNeutralMode(NeutralMode.Brake);
	}
	
	public void brakeOff() {
		driveMotor.setIdleMode(IdleMode.kCoast);
		drivePID.setReference(0 , ControlType.kVelocity);
		turnMotor.setNeutralMode(NeutralMode.Coast);	
	}

	private double toDeg(int units){
		double angle = wrapUnits(units); 
		angle *= 360.0 / 24576.0;
		return angle;
	}

	private int toNativeUnits(double angle){
		int units = (int) angle;
		units *= 24576/360;
		return units;
	}

	private int wrapUnits(int units){
		int offset = Math.abs(units);
		offset %= 24576;
		return offset;
	}

	public boolean onTarget() {
		return Math.abs(last_error) < m_value;
	  }

	public void reset(){
		last_error = 0;
	}

}
