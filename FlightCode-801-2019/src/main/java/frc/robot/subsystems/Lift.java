/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.commands.Lift.LiftStopCMD;
import frc.robot.commands.Lift.LiftUpDownToggleCMD;


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

  public static double rightLiftEncoderInitCount, leftLiftEncoderInitCount;

  private final double motorIsMovingThreshold = 1.0;
  private String LiftTitle = "Lift Motor";
  private NetworkTableEntry rightEncoderPos,leftEncoderPos,setPoint_lift; 
  private NetworkTableEntry kP_lift, kI_lift, kD_lift, kIz_lift, kFF_lift, kMaxOutput_lift;
  private NetworkTableEntry maxVel_lift, minVel_lift, maxAcc_lift, encoderError;
  private ShuffleboardTab tab = Shuffleboard.getTab(LiftTitle);
  
  private final int smartMotionSlot = 0;
  private final int maxEncoderError = 0;
  private double kP = 0.001;
  private double kD = 0.0;
  private double kI = 0.0;
  private double kIz = 0.0;
  private double kFF = 0.0;
  private double kMaxOutput = 1.0;
  private double maxVel = 5700;
  private double maxAcc = 7500;
  private double minVel = 0;
  
  public void init(){
      rightLiftMotor = new CANSparkMax(Constants.rightLiftMotorID, MotorType.kBrushless);
      leftLiftMotor = new CANSparkMax(Constants.leftLiftMotorID, MotorType.kBrushless);
      rightLiftMotor.restoreFactoryDefaults();
      leftLiftMotor.restoreFactoryDefaults();

      rightLiftEncoder = rightLiftMotor.getEncoder();
      leftLiftEncoder = leftLiftMotor.getEncoder();

      rightLiftPID = rightLiftMotor.getPIDController();
      leftLiftPID =leftLiftMotor.getPIDController();

      rightLiftEncoder.setPosition(0.0); //it is assumed that the lift starts in the retracted position
      leftLiftEncoder.setPosition(0.0);

      rightLiftEncoder.setPositionConversionFactor(0.1); //10 shat rotations per 1"
      leftLiftEncoder.setPositionConversionFactor(0.1); //10 shat rotations per 1"

      ///Shuffle Board Start////
      initDashboard();
      // initNetWorkVars();

      //Set PID and Motion Constants
      updatePID();
      updateSmartMotion();
  }

  public void initDashboard(){
    setPoint_lift = tab.add("SetPoint", 10.0).getEntry(); //input is in Inches
    // PID Constants Area
    ShuffleboardLayout LiftMotorPID = Shuffleboard.getTab(LiftTitle)
      .getLayout("ArmMotorPID", BuiltInLayouts.kList)
      .withSize(2, 5)
      .withPosition(0, 0);
    kP_lift = LiftMotorPID.add("kP", kP).withPosition(0, 0).getEntry();
    kI_lift = LiftMotorPID.add("kI_lift", kI).withPosition(0, 1).getEntry();
    kD_lift = LiftMotorPID.add("kD_lift", kD).withPosition(0, 2).getEntry();
    kIz_lift = LiftMotorPID.add("kIz_liftz", kIz).withPosition(0, 3).getEntry(); 
    kFF_lift = LiftMotorPID.add("kFF_lift", kFF).withPosition(0, 4).getEntry(); 
    kMaxOutput_lift = LiftMotorPID.add("kMaxOutput", kMaxOutput).withWidget(BuiltInWidgets.kNumberSlider) 
    .withProperties(Map.of("min", 0, "max", 1)).withPosition(0, 5).getEntry();
    // Smart Motion Coefficients
    // Motion Magic Constants Area
    ShuffleboardLayout LiftMotorMotion = Shuffleboard.getTab(LiftTitle)
    .getLayout("ArmMotorMP", BuiltInLayouts.kList)
    .withSize(2, 5)
    .withPosition(2, 0);
    maxVel_lift = LiftMotorMotion.add("maxVel(RPM)", maxVel).withPosition(0, 0).getEntry(); // rpm
    maxAcc_lift = LiftMotorMotion.add("maxAcc", maxAcc).withPosition(0, 1).getEntry(); //accelaeration
    minVel_lift = LiftMotorMotion.add("minVel", minVel).withPosition(0, 2).getEntry();
    leftEncoderPos = LiftMotorMotion.add("LeftEncPos", 0).withPosition(0, 3).getEntry();
    rightEncoderPos = LiftMotorMotion.add("rightEncoderPos", 0).withPosition(0, 4).getEntry();
    encoderError = LiftMotorMotion.add("LRencoderDelta", 0).withPosition(0, 5).getEntry();


    tab.add(new LiftUpDownToggleCMD()).withPosition(2, 2).withSize(2, 1);    
  }
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
  public void updateSmartMotion(){
    rightLiftPID.setSmartMotionMaxVelocity(maxVel_lift.getDouble(2000), smartMotionSlot);
    rightLiftPID.setSmartMotionMinOutputVelocity(minVel_lift.getDouble(10), smartMotionSlot);
    rightLiftPID.setSmartMotionMaxAccel(maxAcc_lift.getDouble(1500), smartMotionSlot);
    rightLiftPID.setSmartMotionAllowedClosedLoopError(maxEncoderError, smartMotionSlot);

    leftLiftPID.setSmartMotionMaxVelocity(maxVel_lift.getDouble(2000), smartMotionSlot);
    leftLiftPID.setSmartMotionMinOutputVelocity(minVel_lift.getDouble(0), smartMotionSlot);
    leftLiftPID.setSmartMotionMaxAccel(maxAcc_lift.getDouble(1500), smartMotionSlot);
    leftLiftPID.setSmartMotionAllowedClosedLoopError(maxEncoderError, smartMotionSlot);
  }

  public void updatePID(){
    rightLiftPID.setP(kP_lift.getDouble(kP));
    rightLiftPID.setI(kI_lift.getDouble(kI));
    rightLiftPID.setD(kD_lift.getDouble(kD));
    rightLiftPID.setIZone(kIz_lift.getDouble(kIz));
    rightLiftPID.setFF(kFF_lift.getDouble(kFF));
    rightLiftPID.setOutputRange(-kMaxOutput_lift.getDouble(kMaxOutput), kMaxOutput_lift.getDouble(kMaxOutput));

    leftLiftPID.setP(kP_lift.getDouble(0.0005));
    leftLiftPID.setI(kI_lift.getDouble(1e-6));
    leftLiftPID.setD(kD_lift.getDouble(0.0));
    leftLiftPID.setI(kI_lift.getDouble(0.0));
    leftLiftPID.setIZone(kIz_lift.getDouble(0.0));
    leftLiftPID.setFF(kFF_lift.getDouble(0.0));
    leftLiftPID.setOutputRange(-kMaxOutput_lift.getDouble(kMaxOutput), kMaxOutput_lift.getDouble(kMaxOutput));
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new LiftStopCMD());
  }

  public void liftToggle()
  {
      double val = setPoint_lift.getDouble(0.0);
      encoderPos();
      rightLiftPID.setReference(val, ControlType.kSmartMotion);
      leftLiftPID.setReference(val, ControlType.kSmartMotion);
  }

  public boolean isDeltaPosition()
  {
    // if either motor is moving faster than motorIsMovingThreshold it is moving...
    double error = rightLiftEncoder.getPosition() - leftLiftEncoder.getPosition();
    encoderError.setNumber(error);
    return ( Math.abs(error) >  motorIsMovingThreshold);
  }

  public void encoderPos(){
    leftEncoderPos.setNumber(leftLiftEncoder.getPosition());
    rightEncoderPos.setNumber(rightLiftEncoder.getPosition());
  }

  public void stop() {
    rightLiftMotor.stopMotor();
    leftLiftMotor.stopMotor();
  }

  //used to lift to a specified setPoint
  public void lift(double setPoint) {
    rightLiftPID.setReference(setPoint, ControlType.kSmartMotion);
    leftLiftPID.setReference(setPoint, ControlType.kSmartMotion);
  }
  
}
