/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.commands.Elevator.ElevatorBottomCMD;
import frc.robot.commands.Elevator.ElevatorStopCMD;
import frc.robot.commands.Elevator.ElevatorUpperPortCMD;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/* 
 * 
 */ 
public class Elevator extends Subsystem 
{
  public static CANSparkMax rightInsideElevatorMotor;
  public static CANPIDController rightInsideElevatorMotorPID;
  public static CANEncoder rightInsideElevatorMotorEncoder;

  public static CANSparkMax leftElevatorCarriageMotor;
  public static CANPIDController leftElevatorCarriageMotorPID;
  public static CANEncoder leftElevatorCarriageMotorEncoder;

  private String ElevatorTitle = "Elevator";
  private ShuffleboardTab tab = Shuffleboard.getTab(ElevatorTitle);

  private NetworkTableEntry kP_Elevator;
  private NetworkTableEntry kI_Elevator;
  private NetworkTableEntry kD_Elevator;
  private NetworkTableEntry kIz_Elevator;
  private NetworkTableEntry kFF_Elevator;
  private NetworkTableEntry kMaxOutput_Elevator;

  private NetworkTableEntry kP_Carriage;
  private NetworkTableEntry kI_Carriage;
  private NetworkTableEntry kD_Carriage;
  private NetworkTableEntry kIz_Carriage;
  private NetworkTableEntry kFF_Carriage;
  private NetworkTableEntry kMaxOutput_Carriage;

  private NetworkTableEntry maxVel_Elevator;
  private NetworkTableEntry minVel_Elevator;
  private NetworkTableEntry maxAcc_Elevator;
  
  private NetworkTableEntry maxVel_Carriage;
  private NetworkTableEntry minVel_Carriage;
  private NetworkTableEntry maxAcc_Carriage;

  private NetworkTableEntry elevatorEncoderPos;
  private NetworkTableEntry carriageEncoderPos;
  private NetworkTableEntry setPoint_Elevator; 
  private NetworkTableEntry setPoint_Carriage; 
  
  private final int smartMotionSlot = 0;
  private final int maxEncoderError = 0;
  private final int closeEnough = 5;


  public void init()
  {
    // initialize motor
    rightInsideElevatorMotor = new CANSparkMax(Constants.rightInsideElevatorMotorID, MotorType.kBrushless);
    leftElevatorCarriageMotor = new CANSparkMax(Constants.leftElevatorCarriageMotorID, MotorType.kBrushless);
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    rightInsideElevatorMotor.restoreFactoryDefaults();
    leftElevatorCarriageMotor.restoreFactoryDefaults();
    // initialze PID controller and encoder objects
    rightInsideElevatorMotorPID = rightInsideElevatorMotor.getPIDController();
    rightInsideElevatorMotorEncoder = rightInsideElevatorMotor.getEncoder();
    leftElevatorCarriageMotorPID = leftElevatorCarriageMotor.getPIDController();
    leftElevatorCarriageMotorEncoder = leftElevatorCarriageMotor.getEncoder();
    rightInsideElevatorMotorEncoder.setPosition(0.0);
    leftElevatorCarriageMotorEncoder.setPosition(0.0);
    rightInsideElevatorMotorEncoder.setPositionConversionFactor(1.0);
    leftElevatorCarriageMotorEncoder.setPositionConversionFactor(1.0);
    ///Shuffle Board Start////
    initDashboard();
    // initNetWorkVars();
    //Set PID and Motion Constants
    updatePID();
    updateSmartMotion();
  }
  
  
    public void initDashboard()
  {
  //ElevatorMotor
  ShuffleboardLayout ElevatorMotorPID = Shuffleboard.getTab(ElevatorTitle)
    .getLayout("ElevatorMotorPID", BuiltInLayouts.kList)
    .withSize(2, 5)
    .withPosition(0, 0);
  kP_Elevator = ElevatorMotorPID.add("kP_Elevator", .0005).getEntry();
  kI_Elevator = ElevatorMotorPID.add("kI_Elevator", 1e-6).getEntry();
  kD_Elevator = ElevatorMotorPID.add("kD_Elevator", 0).getEntry();
  kIz_Elevator = ElevatorMotorPID.add("kIz_Elevator", 0).getEntry(); 
  kFF_Elevator = ElevatorMotorPID.add("kFF_Elevator", 0).getEntry(); 
  kMaxOutput_Elevator = ElevatorMotorPID.add("kMaxOutput_Elevator", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
  
  ShuffleboardLayout ElevatorMotorMP = Shuffleboard.getTab(ElevatorTitle)
    .getLayout("ElevatorMotorPM", BuiltInLayouts.kList)
    .withSize(2, 3)
    .withPosition(0, 7);
  maxVel_Elevator = ElevatorMotorMP.add("maxVel_Elevator", 5700).getEntry();
  minVel_Elevator = ElevatorMotorMP.add("maxVel_Elevator", 10).getEntry();
  maxAcc_Elevator = ElevatorMotorMP.add("maxAcc_Elevator", 7500).getEntry();
  setPoint_Elevator = ElevatorMotorMP.add("ElevatorSetPos", 0).getEntry();
  elevatorEncoderPos = ElevatorMotorMP.add("ElevatorGetPos", 0).getEntry();

  ElevatorMotorMP.add("SendNewPosition", new ElevatorBottomCMD()); 
  
  
  //CarriageMotor
  ShuffleboardLayout CarriageMotorPID = Shuffleboard.getTab(ElevatorTitle)
   .getLayout("CarriageMotorPID", BuiltInLayouts.kList)
   .withSize(2, 5)
   .withPosition(2, 0);
  kP_Carriage = CarriageMotorPID.add("kP_Carriage", 0.01).getEntry();
  kI_Carriage = CarriageMotorPID.add("kI_Carriage", 0.01).getEntry();
  kD_Carriage = CarriageMotorPID.add("kD_Carriage", 0).getEntry();
  kIz_Carriage = CarriageMotorPID.add("kIz_Carriage", 0).getEntry(); 
  kFF_Carriage = CarriageMotorPID.add("kFF_Carriage", 0.0001).getEntry(); 
  kMaxOutput_Carriage = CarriageMotorPID.add("kMaxOutput_Carriage", 1.0).getEntry(); 
  
  ShuffleboardLayout CarriageMotorMP = Shuffleboard.getTab(ElevatorTitle)
    .getLayout("CarriageMotorPM", BuiltInLayouts.kList)
    .withSize(2, 3)
    .withPosition(2, 7);
  maxVel_Carriage = CarriageMotorMP.add("maxVel_Carriage", 5700).getEntry();
  minVel_Carriage = CarriageMotorMP.add("maxVel_Carriage", 10).getEntry();
  maxAcc_Carriage = CarriageMotorMP.add("maxAcc_Carriage", 7500).getEntry();
  setPoint_Carriage = CarriageMotorMP.add("CarriageSetPos", 0).getEntry();
  carriageEncoderPos = CarriageMotorMP.add("CarriageGetPos", 0).getEntry();

  CarriageMotorMP.add("SendNewPosition", new ElevatorUpperPortCMD()); 

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
    rightInsideElevatorMotorPID.setSmartMotionMaxVelocity(maxVel_Elevator.getDouble(2000), smartMotionSlot);
    rightInsideElevatorMotorPID.setSmartMotionMinOutputVelocity(minVel_Elevator.getDouble(10), smartMotionSlot);
    rightInsideElevatorMotorPID.setSmartMotionMaxAccel(maxAcc_Elevator.getDouble(1500), smartMotionSlot);
    rightInsideElevatorMotorPID.setSmartMotionAllowedClosedLoopError( maxEncoderError, smartMotionSlot);
  
    leftElevatorCarriageMotorPID.setSmartMotionMaxVelocity(maxVel_Carriage.getDouble(2000), smartMotionSlot);
    leftElevatorCarriageMotorPID.setSmartMotionMinOutputVelocity(minVel_Carriage.getDouble(0), smartMotionSlot);
    leftElevatorCarriageMotorPID.setSmartMotionMaxAccel(maxAcc_Carriage.getDouble(1500), smartMotionSlot);
    leftElevatorCarriageMotorPID.setSmartMotionAllowedClosedLoopError( maxEncoderError, smartMotionSlot);
  }
  
  public void updatePID()
  {
    rightInsideElevatorMotorPID.setP(kP_Elevator.getDouble(0.0005));
    rightInsideElevatorMotorPID.setI(kI_Elevator.getDouble(1e-6));
    rightInsideElevatorMotorPID.setD(kD_Elevator.getDouble(0.0));
    rightInsideElevatorMotorPID.setIZone(kIz_Elevator.getDouble(0.0));
    rightInsideElevatorMotorPID.setFF(kFF_Elevator.getDouble(0.0));
    rightInsideElevatorMotorPID.setOutputRange(-kMaxOutput_Carriage.getDouble(1.0), kMaxOutput_Elevator.getDouble(1.0));
  
    leftElevatorCarriageMotorPID.setP(kP_Carriage.getDouble(0.0005));
    leftElevatorCarriageMotorPID.setI(kI_Carriage.getDouble(1e-6));
    leftElevatorCarriageMotorPID.setD(kD_Carriage.getDouble(0.0));
    leftElevatorCarriageMotorPID.setIZone(kIz_Carriage.getDouble(0.0));
    leftElevatorCarriageMotorPID.setFF(kFF_Carriage.getDouble(0.0));
    leftElevatorCarriageMotorPID.setOutputRange(-kMaxOutput_Carriage.getDouble(1.0), kMaxOutput_Carriage.getDouble(1.0));
  }
  
  
  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new ElevatorStopCMD());
  }
  
   public void elevatorRun()
  {
      double setPotint = setPoint_Elevator.getDouble(0.0);
      elevatorEncoderPos();
      rightInsideElevatorMotorPID.setReference(setPotint, ControlType.kSmartMotion);;
  } 

  public void carriageRun()
  {
      double setPotint = setPoint_Carriage.getDouble(0.0);
      elevatorEncoderPos();
      leftElevatorCarriageMotorPID.setReference(setPotint, ControlType.kSmartMotion);;
  } 
  
  public void elevatorEncoderPos()
  {
    elevatorEncoderPos.setNumber(rightInsideElevatorMotorEncoder.getPosition());
  }

  public void carriageEncoderPos()
  {
    carriageEncoderPos.setNumber(leftElevatorCarriageMotorEncoder.getPosition());
  }

  public boolean elevatorIsMoving()
  {
    return Math.abs(rightInsideElevatorMotorEncoder.getPosition() - setPoint_Elevator.getDouble(0.0) ) > closeEnough; 
  }

  public boolean carriageIsMoving()
  {
    return Math.abs(leftElevatorCarriageMotorEncoder.getPosition() - setPoint_Carriage.getDouble(0.0) ) > closeEnough;
  }

  public void stop() {
    rightInsideElevatorMotor.stopMotor();
    leftElevatorCarriageMotor.stopMotor();;
  } 
}




