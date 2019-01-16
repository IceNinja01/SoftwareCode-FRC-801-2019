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

import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;

public class SwervePOD {


	private CANSparkMax driveMotor;
	private CANSparkMax turnMotor;
	private MotorName motorName;
	private CANEncoder driveMotorEnc;
	private CANEncoder turnMotorEnc;
	private CANPIDController drivePID;
	private CANPIDController turnPID;
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
	 * @param kMinOutput the minimum percentage to write to the output, -1 min
	 * @param kMaxOutput the maximum percentage to write to the output, 1 max
	 */
	public void configPIDTurn(double kP, double kI, double kD, double kIz, double kFF, double kMinOutput, double kMaxOutput) {
	    // set PID coefficients for turn motor
		turnPID.setP(kP);
		turnPID.setI(kI);
		turnPID.setD(kD);
		turnPID.setIZone(kIz);
		turnPID.setFF(kFF);
		turnPID.setOutputRange(kMinOutput, kMaxOutput);
					
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
		   int motorNumber = turnMotor.getDeviceId();
		   // Convert rotations to degrees	   
		   double degrees = turnMotorEnc.getPosition(); //will need to add gear ratio here
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
		this.angle = angle * Constants.rotations_per_deg;
		turnPID.setReference(angle, ControlType.kPosition);
	}
	
	public void setDriveEncoder(int counts_per_rev){
		driveMotor.setParameter(CANSparkMaxLowLevel.ConfigParameter.kEncoderCountsPerRev, counts_per_rev);
	}

	public void setTurnEncoder(int counts_per_rev){
		turnMotor.setParameter(CANSparkMaxLowLevel.ConfigParameter.kEncoderCountsPerRev, counts_per_rev);
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
	 * @param freeLimit The current limit at free speed (5700RPM for NEO).
	 */
	public void setTurnCurrentLimit(int stallLimit, int freeLimit) {
		turnMotor.setSmartCurrentLimit(stallLimit, freeLimit);

	}

	public double getAmps() {
		return driveMotor.getOutputCurrent();
	}
	
	public double getVoltage() {
		return driveMotor.getBusVoltage();
	}

	public double getSpeed(){
		//Add the following constants to make proper speed calculations
		//(kMaxRPM  / 600) * (kSensorUnitsPerRotation / kGearRatio)
		return driveMotorEnc.getVelocity(); 
	}

	public int getPosition(){
		return (int) driveMotorEnc.getPosition();
	}

	public void stop() {
		drivePID.setReference(0 , ControlType.kVelocity);
		driveMotor.stopMotor();
		turnMotor.stopMotor();
	}
	
	public void brakeOn() {
		driveMotor.setIdleMode(IdleMode.kBrake);
		turnMotor.setIdleMode(IdleMode.kBrake);
	}
	
	public void brakeOff() {
		driveMotor.setIdleMode(IdleMode.kCoast);
		drivePID.setReference(0 , ControlType.kVelocity);
		turnMotor.setIdleMode(IdleMode.kCoast);
		
	}


}
