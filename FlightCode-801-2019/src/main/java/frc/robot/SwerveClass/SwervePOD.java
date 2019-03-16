package frc.robot.SwerveClass;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.hal.sim.mockdata.RoboRioDataJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utilities.PID;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

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

	private int nativeUnits;

	private double last_error;

	private int m_value;

	private boolean kMotorInvert = false;
	private boolean kSensorPhase = true;

	private boolean d_motorInvert = false;

	// private int setUnits;

	// private int delta;

	// private int setUnitsWrap;

	// private int setAngle;

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
		turnMotorPID.setMaxIOutput((double) kIz);
		turnMotorPID.setOutputLimits(kMaxOutput);
		turnMotorPID.setContinous(true);
		turnMotorPID.setContinousInputRange(360);

		// /* set the peak and nominal outputs, 12V means full */
		turnMotor.configNominalOutputForward(0, 10);
		turnMotor.configNominalOutputReverse(0, 10);
		turnMotor.configPeakOutputForward(kMaxOutput, 10);
		turnMotor.configPeakOutputReverse(kMinOutput, 10);
		// /* 0.001 represents 0.1% - default value is 0.04 or 4% */
		//  turnMotor.configNeutralDeadband(0.001, 10);
		// /**
		//  * Grab the 360 degree position of the MagEncoder's absolute
		//  * position, and intitally set the relative sensor to match.
		//  */

// 		// //set coast mode
		turnMotor.setNeutralMode(NeutralMode.Coast);
		turnMotor.setInverted(kMotorInvert);
//		//set Voltage for turn motors
		turnMotor.set(ControlMode.PercentOutput, 0.0);
		setBias();


	}

	public void setBias(){
		int absolutePosition = getAbsAngle();	

		turnMotor.setSelectedSensorPosition(0, 0, 10);
		// if (absolutePosition > Constants.AngleBias[0])
		// {
		// 	turnMotor.setSelectedSensorPosition(24576 - (absolutePosition-Constants.AngleBias[motorName.ordinal()]), 0, 10);
		// }
		// else
		// {
		// 	turnMotor.setSelectedSensorPosition(Constants.AngleBias[motorName.ordinal()]-absolutePosition, 0, 10);
		// }
	}

	public void setInvertTurn(boolean invert){
		turnMotor.setInverted(invert);
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
		driveMotor.setInverted(d_motorInvert);
		driveMotor.setIdleMode(IdleMode.kCoast);
	}

	public void invertDriveMotor(boolean invert){
		d_motorInvert = invert;
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
		// SmartDashboard.putNumber("TargetSpeed " + motorName, speed);	
		driveMotor.set(speed);
		// drivePID.setReference(speed, ControlType.kVelocity);
		// SmartDashboard.putNumber("OutPut PWM " + motorName, driveMotor.getAppliedOutput());	
		// SmartDashboard.putNumber("Motor PID Error" + motorName, driveMotorEnc.getVelocity()-speed);

	}

	public double getTurnPIDError(){
		// double error = pidTurnController.getError();
		double error = turnMotorPID.getError();

		return error;
	}

	// public void getPIDOut(){

	// 	SmartDashboard.putNumber("Turn_PIDOut", turnMotorPID.getOutput());
	// }
	
	public void setAngle(double angle) {
		// Set new position of motor

		//angle = Math.toDegrees(Math.atan2(Robot.oi.manipulator.getRawAxis(0), Robot.oi.manipulator.getRawAxis(1)));
		turnMotor.set(ControlMode.PercentOutput, turnMotorPID.getOutput(getAngleDeg(), angle));


	}

	public void disablePIDTurn(){
		turnMotor.set(ControlMode.PercentOutput, 0.0);
		turnMotorPID.reset();
	}


	// public int getSetNativeUnits(double setAng){
	// 	setAngle = toNativeUnits(setAng);
	// 	setUnitsWrap = wrapUnits(getNativeUnits());
	// 	delta = setAngle - setUnitsWrap;
	// 	setUnits = 0;
	// 	if(delta>0){
	// 		setUnits = getNativeUnits() + delta;
	// 	}
	// 	else{
	// 		setUnits = getNativeUnits() - delta;
	// 	}
	// 	SmartDashboard.putNumber("1_SetUnits", setUnits);
	// 	SmartDashboard.putNumber("1_Delta", delta);
	// 	SmartDashboard.putNumber("1_setUnitsWrap", setUnitsWrap);
	// 	SmartDashboard.putNumber("1_setAngleUnits", setAngle);

	// 	return setUnits;
	// }

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

	public double getTurnVoltage(){
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
		SmartDashboard.putNumber("Motor Speed" + motorName, speed);
		return speed;
	}

	public double getPosition(){
		double position = driveMotorEnc.getPosition();
		position = (position/5.1)*(4*Math.PI);
		return position;
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
		int offset = (units) + 24576;
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
