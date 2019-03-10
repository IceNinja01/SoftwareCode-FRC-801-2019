/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;


/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax rightLiftMotor;
  private CANEncoder rightLiftEncoder;
  private CANPIDController rightLiftPID; 
  private CANSparkMax leftLiftMotor;
  private CANEncoder leftLiftEncoder;
  private CANPIDController leftLiftPID; 

  public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  public static double rightLiftEncoderInitCount, leftLiftEncoderInitCount;

  private final double motorIsMovingThreshold = 0.1;

  public void init(){
      rightLiftMotor = new CANSparkMax(Constants.rightLiftMotorID, MotorType.kBrushless);
      leftLiftMotor = new CANSparkMax(Constants.leftLiftMotorID, MotorType.kBrushless);

      rightLiftEncoder = rightLiftMotor.getEncoder();
      leftLiftEncoder = leftLiftMotor.getEncoder();

      rightLiftPID = rightLiftMotor.getPIDController();
      leftLiftPID =leftLiftMotor.getPIDController();

      rightLiftMotor.restoreFactoryDefaults();
      leftLiftMotor.restoreFactoryDefaults();

      // PID coefficients
      kP = 5e-5; 
      kI = 1e-6;
      kD = 0; 
      kIz = 0; 
      kFF = 0; 
      kMaxOutput = 1; 
      kMinOutput = -1;
      maxRPM = 5700;
  
      // Smart Motion Coefficients
      maxVel = 2000; // rpm
      maxAcc = 1500;
  
      // set PID coefficients
      rightLiftPID.setP(kP);
      rightLiftPID.setI(kI);
      rightLiftPID.setD(kD);
      rightLiftPID.setIZone(kIz);
      rightLiftPID.setFF(kFF);
      rightLiftPID.setOutputRange(kMinOutput, kMaxOutput);
      leftLiftPID.setP(kP);
      leftLiftPID.setI(kI);
      leftLiftPID.setD(kD);
      leftLiftPID.setIZone(kIz);
      leftLiftPID.setFF(kFF);
      leftLiftPID.setOutputRange(kMinOutput, kMaxOutput);

      /**
       * Smart Motion coefficients are set on a CANPIDController object
       * 
       * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
       * the pid controller in Smart Motion mode
       * - setSmartMotionMinOutputVelocity() will put a lower bound in
       * RPM of the pid controller in Smart Motion mode
       * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
       * of the pid controller in Smart Motion mode
       * - setSmartMotionAllowedClosedLoopError() will set the max allowed
       * error for the pid controller in Smart Motion mode
       */
      int smartMotionSlot = 0;
      rightLiftPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      rightLiftPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
      rightLiftPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      rightLiftPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
      leftLiftPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      leftLiftPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
      leftLiftPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      leftLiftPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

      rightLiftEncoderInitCount = rightLiftEncoder.getPosition();
      leftLiftEncoderInitCount = rightLiftEncoder.getPosition();
  }

  public void liftToggle()
  {
    if(!isMoving())
    {
      // this is just for testing... not operational yet..
      double setPoint = rightLiftEncoder.getPosition() + 100;
      rightLiftPID.setReference(setPoint, ControlType.kSmartMotion);
    }
  }

  public boolean isMoving()
  {
    // if either motor is moving faster than motorIsMovingThreshold it is moving...
    return ( rightLiftEncoder.getVelocity() >  motorIsMovingThreshold
            || leftLiftEncoder.getVelocity() >  motorIsMovingThreshold );
  }


  @Override
  public void initDefaultCommand() {  }

  public void stop() {
    rightLiftMotor.stopMotor();
    leftLiftMotor.stopMotor();
  }
  
}
