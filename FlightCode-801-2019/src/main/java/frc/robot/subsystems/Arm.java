/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.SwerveClass.SwervePOD.MotorName;
import frc.robot.Utilities.PID;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;




/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private MotorName motorName;
  private TalonSRX armMotor;
  private PID armMotorPID;
  

  /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;


  /** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;

  public void init(){
    //  rightLiftMotor = new CANSparkMax(Constants.rightLiftMotorID, MotorType.kBrushless);
    armMotor  = new TalonSRX(Constants.ArmPositionMotorID);	
		this.motorName = motorName;
  
		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		armMotor.setSensorPhase(true);
		armMotor.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* Set the peak and nominal outputs */
		armMotor.configNominalOutputForward(0, kTimeoutMs);
		armMotor.configNominalOutputReverse(0, kTimeoutMs);
		armMotor.configPeakOutputForward(1, kTimeoutMs);
		armMotor.configPeakOutputReverse(-1, kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		armMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		armMotor.config_kF(kSlotIdx, kGains.kF, kTimeoutMs);
		armMotor.config_kP(kSlotIdx, kGains.kP, kTimeoutMs);
		armMotor.config_kI(kSlotIdx, kGains.kI, kTimeoutMs);
		armMotor.config_kD(kSlotIdx, kGains.kD, kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		armMotor.configMotionCruiseVelocity(15000, kTimeoutMs);
		armMotor.configMotionAcceleration(6000, kTimeoutMs);

		/* Zero the sensor */
		armMotor.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

		/* Factory default hardware to prevent unexpected behavior */
		armMotor.configFactoryDefault();  
  }

  @Override
  public void initDefaultCommand() {
 //   setDefaultCommand(new LiftStop());
  }

  public void stop() {
 //   rightLiftMotor.stopMotor();
  }
  
}
