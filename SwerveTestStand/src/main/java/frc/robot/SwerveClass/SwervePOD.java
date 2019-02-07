package frc.robot.SwerveClass;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Utilities.Utils;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;

public class SwervePOD {

	private MotorName motorName;

	private CANSparkMax driveMotor;
	private CANEncoder driveMotorEnc;
	private CANPIDController drivePID;

	private Team801TalonSRX turnMotor;
	private PIDSource pidTurnSource;
	private PIDController pidTurnController;

	private double k_drive_P, k_drive_I, k_drive_D, k_drive_Iz, k_drive_FF, kMinRPM, kMaxRPM;
	private double k_turn_P, k_turn_I, k_turn_D, k_turn_Iz, k_turn_FF, kMinAngle, kMaxAngle;
	private double angle;

	private int m_inputRange = 360;

	private int nativeUnits;

	private double last_error;

	private double turn_kP;

	private double turn_kD;
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

		turnMotor  = new Team801TalonSRX(Turn);		
		// motorName.value = PODName;
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
		turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
		turnMotor.setSensorPhase(true); 
		turn_kP = kP;
		turn_kD = kD;

		// turnMotor.config_kP(0, kP, 30);
		// turnMotor.config_kI(0, kI, 30);
		// turnMotor.config_kD(0, kD, 30);
		// turnMotor.config_IntegralZone(0, kIz, 30);
		// turnMotor.config_kF(0, kFF, 30);
		
		// /* set the peak and nominal outputs, 12V means full */
		turnMotor.configNominalOutputForward(0, 30);
		turnMotor.configNominalOutputReverse(0, 30);
		turnMotor.configPeakOutputForward(11.0, 30);
		turnMotor.configPeakOutputReverse(-11.0, 30);
		turnMotor.enableVoltageCompensation(true); 
		// /* 0.001 represents 0.1% - default value is 0.04 or 4% */
		// turnMotor.configNeutralDeadband(0.001, 30);
		// // turnMotor.configAllowableClosedloopError(0, deadBand*(24576/360), 30); // degrees deadband
		// /**
		//  * Grab the 360 degree position of the MagEncoder's absolute
		//  * position, and intitally set the relative sensor to match.
		//  */
		int absolutePosition = turnMotor.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (true) { absolutePosition *= -1; }
		if (false) { absolutePosition *= -1; }
		SmartDashboard.putNumber("PositionEnc", absolutePosition);	
		turnMotor.setSelectedSensorPosition(0, 0, 30);
		// //set coast mode
		turnMotor.setNeutralMode(NeutralMode.Coast);
		turnMotor.setInverted(false);
		/* 0.001 represents 0.1% - default value is 0.04 or 4% */
		turnMotor.configNeutralDeadband(0.001, 10);
//		//set Voltage for turn motors
		turnMotor.set(ControlMode.PercentOutput, 0.0);

		// pidTurnSource = new PIDSource() {				
		// 	@Override
		// 	public void setPIDSourceType(PIDSourceType pidSource) {				
		// 	}
		// 	@Override
		// 	public double pidGet() {
		// 		return getAngleUnits();
		// 	}				
		// 	@Override
		// 	public PIDSourceType getPIDSourceType() {
		// 		return PIDSourceType.kDisplacement;
		// 	}
		// };
		// pidTurnController = new PIDController(kP, kI, kD, pidTurnSource, turnMotor);
		// pidTurnController.setAbsoluteTolerance(deadBand);
		// pidTurnController.setInputRange(0, 360);
		// pidTurnController.setContinuous(true);
		// pidTurnController.setOutputRange(-kMinOutput, kMaxOutput);
		// pidTurnController.enable();
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
	}

	public int getAngleUnits(){
		nativeUnits = turnMotor.getSelectedSensorPosition(0);
		return nativeUnits;
	}

	public double getAngleDeg() {
		int motorNumber = turnMotor.getDeviceID();
		// Convert rotations to degrees	   
		nativeUnits = wrapUnits(getAngleUnits());
		SmartDashboard.putNumber("nativeUnits", nativeUnits);
		double degrees = toDeg(nativeUnits) - Constants.AngleBias[0];
		SmartDashboard.putNumber("AngleEncoder", degrees);
		return degrees;
	}
	
	public void setSpeed(double speed) {
		drivePID.setReference(speed, ControlType.kVelocity);
	}
	
	public void setAngle(double angle) {
		// pidTurnController.setSetpoint(angle);
		double error = getError(angle);
		// Set new position of motor
		turnMotor.set(ControlMode.PercentOutput, error);
	}
	
	private double getError(double angle2) {
		int relativeUnits = nativeUnits;
		SmartDashboard.putNumber("Relative units", relativeUnits);
		double error = angle2 - getAngleDeg();
		int setPoint = relativeUnits;
		error %= m_inputRange;
		if (Math.abs(error) > m_inputRange / 2) { // if going from 10 -> 350, you must calculate the difference
		  if (error > 0) {
			error -= m_inputRange;
		  } else {
			error += m_inputRange;
		  }
		}

		// if (error > 0){ // add or subtract error to current position
		// 	error += relativeUnits;
		// 	}
		// else{
		// 	error -= relativeUnits;
		// } 
		double pidOutPut = turn_kP * error + turn_kD * last_error;
		last_error = error - last_error;
		SmartDashboard.putNumber("setAnlge", angle);
		SmartDashboard.putNumber("Error",  error);
		SmartDashboard.putNumber("SetPoint",  setPoint);

		return pidOutPut;
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
		turnMotor.configPeakCurrentLimit(peakAmps, 30); /* 35 A */
		turnMotor.configPeakCurrentDuration(durationMs, 30); /* 200ms */
		turnMotor.configContinuousCurrentLimit(continousAmps, 30); /* 30A */
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
		speed = (speed/5.33)*(4*Math.PI)*(1/60.0)*(1/12.0);
		SmartDashboard.putNumber("Motor Speed", speed);
		return speed;
	}

	public int getPosition(){
		return (int) driveMotorEnc.getPosition();
	}

	public void stop() {
		drivePID.setReference(0 , ControlType.kVelocity);
		pidTurnController.disable();
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
		int offset = units;
		offset %= 24576;
		return offset;
	}

}
