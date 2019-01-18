package frc.robot.SwerveClass;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

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
	public void configPIDTurn(double kP, double kI, double kD, int kIz, double kFF, double kMinOutput, double kMaxOutput, double deadBand) {
	    // set PID coefficients for turn motor
		turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
		/* set the peak and nominal outputs, 12V means full */
		turnMotor.configNominalOutputForward(0,10);
		turnMotor.configNominalOutputReverse(0, 10);
		turnMotor.configPeakOutputForward(11.0, 10);
		turnMotor.configPeakOutputReverse(-11.0, 10);
		turnMotor.enableVoltageCompensation(true); 
		/* 0.001 represents 0.1% - default value is 0.04 or 4% */
		turnMotor.configNeutralDeadband(0.001, 10);
		//set coast mode
		turnMotor.setNeutralMode(NeutralMode.Coast);
//		//set Voltage for turn motors
		turnMotor.setSensorPhase(false); 
		turnMotor.set(ControlMode.PercentOutput, 0.0);

		pidTurnSource = new PIDSource() {				
			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {				
			}
			@Override
			public double pidGet() {
				return getAngle();
			}				
			@Override
			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}
		};
		pidTurnController = new PIDController(kP, kI, kD, pidTurnSource, turnMotor);
		pidTurnController.setAbsoluteTolerance(deadBand);
		pidTurnController.setInputRange(0, 360);
		pidTurnController.setContinuous(true);
		pidTurnController.setOutputRange(-kMinOutput, kMaxOutput);
		pidTurnController.enable();
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

	public double getAngle() {
		int motorNumber = turnMotor.getDeviceID();
		// Convert rotations to degrees	   
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
		drivePID.setReference(speed, ControlType.kVelocity);
	}
	
	public void setAngle(double angle) {
		pidTurnController.setSetpoint(angle);
	}
	
	public void setDriveEncoder(int counts_per_rev){
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
		turnMotor.configPeakCurrentLimit(peakAmps, 10); /* 35 A */
		turnMotor.configPeakCurrentDuration(durationMs, 10); /* 200ms */
		turnMotor.configContinuousCurrentLimit(continousAmps, 10); /* 30A */
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
		return driveMotorEnc.getVelocity(); 
	}

	public double getPosition(){
		return driveMotorEnc.getPosition();
	}

	public void stop() {
		drivePID.setReference(0 , ControlType.kVelocity);
		pidTurnController.disable();
		turnMotor.set(ControlMode.PercentOutput, 0.0);
	}
	
	public void brakeOn() {
		driveMotor.setIdleMode(IdleMode.kBrake);
		turnMotor.setNeutralMode(NeutralMode.Brake);
		pidTurnController.disable();
	}
	
	public void brakeOff() {
		driveMotor.setIdleMode(IdleMode.kCoast);
		drivePID.setReference(0 , ControlType.kVelocity);
		turnMotor.setNeutralMode(NeutralMode.Coast);	
	}

}
