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
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.commands.Lift.LiftStop;
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
  private NetworkTableEntry encoderError,rightEncoderPos,leftEncoderPos,setPoint_lift; 
  private NetworkTableEntry kP_lift, kI_lift, kD_lift, kIz_lift, kFF_lift, kMaxOutput_lift, kMinOutput_lift;
  private NetworkTableEntry maxRPM_lift, maxVel_lift, minVel_lift, maxAcc_lift, allowedErr_lift;
  private ShuffleboardTab tab = Shuffleboard.getTab(LiftTitle);
  
  private int smartMotionSlot;


  
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

      smartMotionSlot = 0;
      rightLiftEncoderInitCount = rightLiftEncoder.getPosition();
      leftLiftEncoderInitCount = rightLiftEncoder.getPosition();

  }

  public void initDashboard(){
    setPoint_lift = tab.add("SetPoint", 10.0).getEntry(); //input is in Inches
    kP_lift = tab.add("kP", .0005).getEntry();
    kI_lift = tab.add("kI_lift", 1e-6).getEntry();
    kD_lift = tab.add("kD_lift", 0).getEntry();
    kI_lift = tab.add("kI_liftz", 0).getEntry(); 
    kFF_lift = tab.add("kFF_lift", 0).getEntry(); 
    kMaxOutput_lift = tab.add("kMaxOutput", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
     // specify widget properties here
    kMinOutput_lift = tab.add("kMinOutput", -1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 0)).getEntry();
    maxRPM_lift = tab.add("maxRPM", 5700).getEntry();
    // Smart Motion Coefficients
    maxVel_lift = tab.add("maxVel", 5700).getEntry(); // rpm
    maxAcc_lift = tab.add("maxAcc", 7500).getEntry(); //accelaeration
    minVel_lift = tab.add("minVel", 10).getEntry();
    allowedErr_lift = tab.add("allowedErr", 0).getEntry();
    leftEncoderPos = tab.add("LeftEncPos", 0).getEntry();
    rightEncoderPos = tab.add("rightEncoderPos", 0).getEntry();
    encoderError = tab.add("EncoderError", 0).getEntry();

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
    rightLiftPID.setSmartMotionAllowedClosedLoopError(allowedErr_lift.getDouble(0), smartMotionSlot);
    leftLiftPID.setSmartMotionMaxVelocity(maxVel_lift.getDouble(2000), smartMotionSlot);
    leftLiftPID.setSmartMotionMinOutputVelocity(minVel_lift.getDouble(0), smartMotionSlot);
    leftLiftPID.setSmartMotionMaxAccel(maxAcc_lift.getDouble(1500), smartMotionSlot);
    leftLiftPID.setSmartMotionAllowedClosedLoopError(allowedErr_lift.getDouble(1000), smartMotionSlot);
  }

  public void updatePID(){
    rightLiftPID.setP(kP_lift.getDouble(0.0005));
    rightLiftPID.setI(kI_lift.getDouble(1e-6));
    rightLiftPID.setD(kD_lift.getDouble(0.0));
    rightLiftPID.setIZone(kI_lift.getDouble(0.0));
    rightLiftPID.setFF(kFF_lift.getDouble(0.0));
    rightLiftPID.setOutputRange(kMinOutput_lift.getDouble(-1.0), kMaxOutput_lift.getDouble(1.0));
    leftLiftPID.setP(kP_lift.getDouble(0.0005));
    leftLiftPID.setI(kI_lift.getDouble(1e-6));
    leftLiftPID.setD(kD_lift.getDouble(0.0));
    leftLiftPID.setIZone(kI_lift.getDouble(0.0));
    leftLiftPID.setFF(kFF_lift.getDouble(0.0));
    leftLiftPID.setOutputRange(kMinOutput_lift.getDouble(-1.0), kMaxOutput_lift.getDouble(1.0));
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new LiftStop());
  }

  public void liftToggle()
  {
      double val = setPoint_lift.getDouble(10.0);
      encoderPos();
      rightLiftPID.setReference(val, ControlType.kSmartMotion);
      leftLiftPID.setReference(val, ControlType.kSmartMotion);
  }

  public boolean isMoving()
  {
    // if either motor is moving faster than motorIsMovingThreshold it is moving...
    double error =rightLiftEncoder.getPosition() - leftLiftEncoder.getPosition();
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
