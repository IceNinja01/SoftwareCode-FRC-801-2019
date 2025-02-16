/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.commands.Elevator.CarriageManualPositionCMD;
import frc.robot.commands.Elevator.ElevatorBottomCMD;
import frc.robot.commands.Elevator.ElevatorManualPositionCMD;
import frc.robot.commands.Elevator.ElevatorStopCMD;
import frc.robot.commands.Elevator.UpdateElevatorPIDCMD;
import frc.robot.commands.Elevator.UpdateCarriagePIDCMD;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Elevator extends Subsystem 
{
  public static CANSparkMax ElevatorMotor;
  public static CANPIDController rightInsideElevatorMotorPID;
  public static CANEncoder ElevatorMotorEncoder;

  public static CANSparkMax CarriageMotor;
  public static CANPIDController leftElevatorCarriageMotorPID;
  public static CANEncoder CarriageMotorEncoder;

  private String ElevatorTitle = "Elevator";
  private ShuffleboardTab elevatorTab = Shuffleboard.getTab(ElevatorTitle);

  private NetworkTableEntry kP_Elevator;
  private NetworkTableEntry kI_Elevator;
  private NetworkTableEntry kD_Elevator;
  private NetworkTableEntry kIz_Elevator;
  private NetworkTableEntry kFF_Elevator;
  private NetworkTableEntry kMaxOutput_Elevator;
  private NetworkTableEntry kMinOutput_Elevator;

  private NetworkTableEntry kP_Carriage;
  private NetworkTableEntry kI_Carriage;
  private NetworkTableEntry kD_Carriage;
  private NetworkTableEntry kIz_Carriage;
  private NetworkTableEntry kFF_Carriage;
  private NetworkTableEntry kMaxOutput_Carriage;
  private NetworkTableEntry kMinOutput_Carriage;

  private NetworkTableEntry maxVel_Elevator;
  private NetworkTableEntry minVel_Elevator;
  private NetworkTableEntry maxAcc_Elevator;
  
  private NetworkTableEntry maxVel_Carriage;
  private NetworkTableEntry minVel_Carriage;
  private NetworkTableEntry maxAcc_Carriage;

  private NetworkTableEntry elevatorEncoderPos;
  private NetworkTableEntry carriageEncoderPos;
  private NetworkTableEntry newSetPoint_Elevator; 
  private NetworkTableEntry newSetPoint_Carriage; 
  
  private final int smartMotionSlot = 0;
  private final double closeEnough = 0.5;

  private double setPoint_Elevator  = Constants.ElevatorInitPosition; 
  private double setPoint_Carriage = Constants.CarriageInitPosition;

  public void init()
  {
    // initialize motor
    ElevatorMotor = new CANSparkMax(Constants.rightInsideElevatorMotorID, MotorType.kBrushless);
    CarriageMotor = new CANSparkMax(Constants.CarriageMotorID, MotorType.kBrushless);
    // rightInsideElevatorMotor.restoreFactoryDefaults();
    // leftElevatorCarriageMotor.restoreFactoryDefaults();

    ElevatorMotor.setSmartCurrentLimit(70, 60);
    CarriageMotor.setSmartCurrentLimit(50, 50);

    ElevatorMotor.setIdleMode(IdleMode.kCoast);
    CarriageMotor.setIdleMode(IdleMode.kCoast);

    ElevatorMotor.setInverted(true);
    
    // initialze PID controller and encoder objects
    rightInsideElevatorMotorPID = ElevatorMotor.getPIDController();
    ElevatorMotorEncoder = ElevatorMotor.getEncoder();

    leftElevatorCarriageMotorPID = CarriageMotor.getPIDController();
    CarriageMotorEncoder = CarriageMotor.getEncoder();

     //it is assumed that the elevator and carriage start in the fully lowered position
    ElevatorMotorEncoder.setPosition(0.0); 
    CarriageMotorEncoder.setPosition(0.0);

    ElevatorMotorEncoder.setPositionConversionFactor(1.416);
    CarriageMotorEncoder.setPositionConversionFactor(1.416);

    //ElevatorMotor
    ShuffleboardLayout ElevatorMotorPID = elevatorTab
      .getLayout("ElevatorMotorPID", BuiltInLayouts.kList)
      .withSize(2, 8)
      .withPosition(0, 0);
    kP_Elevator = ElevatorMotorPID.add( "kP_Elevator", 0.0 ).getEntry();
    kI_Elevator = ElevatorMotorPID.add( "kI_Elevator", 0.0 ).getEntry();
    kD_Elevator = ElevatorMotorPID.add( "kD_Elevator", 0.0 ).getEntry();
    kIz_Elevator = ElevatorMotorPID.add( "kIz_Elevator", 0.0 ).getEntry(); 
    kFF_Elevator = ElevatorMotorPID.add( "kFF_Elevator", 0.0 ).getEntry(); 
    kMaxOutput_Elevator = ElevatorMotorPID.add( "kMaxOutput_Elevator", 0.0 ).getEntry();
    kMinOutput_Elevator = ElevatorMotorPID.add( "kMinOutput_Elevator", 0.0 ).getEntry();
  
    ElevatorMotorPID.add("UpdatePID_Elevator", new UpdateElevatorPIDCMD()).withPosition(0, 7);
  
    kP_Elevator.setNumber( Constants.ElevatorMotorPID_kP );
    kI_Elevator.setNumber( Constants.ElevatorMotorPID_kI );
    kD_Elevator.setNumber( Constants.ElevatorMotorPID_kD );
    kIz_Elevator.setNumber( Constants.ElevatorMotorPID_kIZone );
    kFF_Elevator.setNumber( Constants.ElevatorMotorPID_kFF );
    kMaxOutput_Elevator.setNumber( Constants.ElevatorMotorPID_kOutputRangeMax );
    kMinOutput_Elevator.setNumber( Constants.ElevatorMotorPID_kOutputRangeMin );
  
    rightInsideElevatorMotorPID.setP( Constants.ElevatorMotorPID_kP );
    rightInsideElevatorMotorPID.setI( Constants.ElevatorMotorPID_kI );
    rightInsideElevatorMotorPID.setD( Constants.ElevatorMotorPID_kD );
    rightInsideElevatorMotorPID.setIZone( Constants.ElevatorMotorPID_kIZone );
    rightInsideElevatorMotorPID.setFF( Constants.ElevatorMotorPID_kFF );
    rightInsideElevatorMotorPID.setOutputRange( Constants.ElevatorMotorPID_kOutputRangeMin, Constants.ElevatorMotorPID_kOutputRangeMax );
  
    ShuffleboardLayout ElevatorMotorMP = Shuffleboard.getTab(ElevatorTitle)
      .getLayout("ElevatorMotorPM", BuiltInLayouts.kList)
      .withSize(2, 8)
      .withPosition(2, 0);
    maxVel_Elevator = ElevatorMotorMP.add( "maxVel_Elevator", 0.0 ).getEntry();
    minVel_Elevator = ElevatorMotorMP.add( "minVel_Elevator", 0.0 ).getEntry();
    maxAcc_Elevator = ElevatorMotorMP.add( "maxAcc_Elevator", 0.0 ).getEntry();
    newSetPoint_Elevator = ElevatorMotorMP.add( "ElevatorSetPos", 0.0 ).getEntry();
    elevatorEncoderPos = ElevatorMotorMP.add( "ElevatorGetPos", 0.0 ).getEntry();
  
    ElevatorMotorMP.add( "SendNewPosition", new ElevatorManualPositionCMD());
  
    maxVel_Elevator.setNumber( Constants.ElevatorMotorMotionMaxVelocity );
    minVel_Elevator.setNumber( Constants.ElevatorMotorMotionMinOutputVelocity );
    maxAcc_Elevator.setNumber( Constants.ElevatorMotorMotionMaxAccel );
    newSetPoint_Elevator.setNumber( Constants.ElevatorInitPosition ); 
  
    rightInsideElevatorMotorPID.setSmartMotionMaxVelocity( Constants.ElevatorMotorMotionMaxVelocity, smartMotionSlot );
    rightInsideElevatorMotorPID.setSmartMotionMinOutputVelocity( Constants.ElevatorMotorMotionMinOutputVelocity, smartMotionSlot );
    rightInsideElevatorMotorPID.setSmartMotionMaxAccel( Constants.ElevatorMotorMotionMaxAccel, smartMotionSlot );
    rightInsideElevatorMotorPID.setSmartMotionAllowedClosedLoopError( Constants.ElevatorMotorMotionAllowedClosedLoopError, smartMotionSlot);
    rightInsideElevatorMotorPID.setReference(Constants.ElevatorInitPosition, ControlType.kPosition);
  
    
    //CarriageMotor
    ShuffleboardLayout CarriageMotorPID = elevatorTab
     .getLayout("CarriageMotorPID", BuiltInLayouts.kList)
     .withSize(2, 8)
     .withPosition(4, 0);
    kP_Carriage = CarriageMotorPID.add( "kP_Carriage", 0.0 ).getEntry();
    kI_Carriage = CarriageMotorPID.add( "kI_Carriage", 0.0 ).getEntry();
    kD_Carriage = CarriageMotorPID.add( "kD_Carriage", 0.0 ).getEntry();
    kIz_Carriage = CarriageMotorPID.add( "kIz_Carriage", 0.0 ).getEntry(); 
    kFF_Carriage = CarriageMotorPID.add( "kFF_Carriage", 0.0 ).getEntry(); 
    kMaxOutput_Carriage = CarriageMotorPID.add( "kMaxOutput_Carriage", 0.0 ).getEntry(); 
    kMinOutput_Carriage = CarriageMotorPID.add( "kMinOutput_Carriage", 0.0 ).getEntry();
  
    CarriageMotorPID.add("UpdatePID_Carriage", new UpdateCarriagePIDCMD() );
  
    kP_Carriage.setNumber( Constants.CarriageMotorPID_kP );
    kI_Carriage.setNumber( Constants.CarriageMotorPID_kI );
    kD_Carriage.setNumber( Constants.CarriageMotorPID_kD );
    kIz_Carriage.setNumber( Constants.CarriageMotorPID_kIZone );
    kFF_Carriage.setNumber( Constants.CarriageMotorPID_kFF );
    kMaxOutput_Carriage.setNumber( Constants.CarriageMotorPID_kOutputRangeMax );
    kMinOutput_Carriage.setNumber( Constants.CarriageMotorPID_kOutputRangeMmin );
    
    leftElevatorCarriageMotorPID.setP( Constants.CarriageMotorPID_kP );
    leftElevatorCarriageMotorPID.setI( Constants.CarriageMotorPID_kI );
    leftElevatorCarriageMotorPID.setD( Constants.CarriageMotorPID_kD );
    leftElevatorCarriageMotorPID.setIZone( Constants.CarriageMotorPID_kIZone );
    leftElevatorCarriageMotorPID.setFF( Constants.CarriageMotorPID_kFF );
    leftElevatorCarriageMotorPID.setOutputRange( Constants.CarriageMotorPID_kOutputRangeMmin, Constants.CarriageMotorPID_kOutputRangeMax );
    
    ShuffleboardLayout CarriageMotorMP = elevatorTab
      .getLayout("CarriageMotorPM", BuiltInLayouts.kList)
      .withSize(2, 8)
      .withPosition(6, 0);
    maxVel_Carriage = CarriageMotorMP.add( "maxVel_Carriage", 0.0 ).getEntry();
    minVel_Carriage = CarriageMotorMP.add( "minVel_Carriage", 0.0 ).getEntry();
    maxAcc_Carriage = CarriageMotorMP.add( "maxAcc_Carriage", 0.0 ).getEntry();
    newSetPoint_Carriage = CarriageMotorMP.add( "CarriageSetPos", 0.0 ).getEntry();
    carriageEncoderPos = CarriageMotorMP.add( "CarriageGetPos", 0.0 ).getEntry();
  
    CarriageMotorMP.add( "SendNewPosition", new CarriageManualPositionCMD() );
  
    maxVel_Carriage.setNumber( Constants.CarriageMotorMotionMaxVelocity );
    minVel_Carriage.setNumber( Constants.ElevatorMotorMotionMinOutputVelocity );
    maxAcc_Carriage.setNumber( Constants.CarriageMotorMotionMaxAccel );
    newSetPoint_Carriage.setNumber( Constants.CarriageInitPosition );

    leftElevatorCarriageMotorPID.setSmartMotionMaxVelocity( Constants.CarriageMotorMotionMaxVelocity, smartMotionSlot );
    leftElevatorCarriageMotorPID.setSmartMotionMinOutputVelocity( Constants.CarriageMotorMotionMinOutputVelocity, smartMotionSlot );
    leftElevatorCarriageMotorPID.setSmartMotionMaxAccel( Constants.ElevatorMotorMotionMaxAccel, smartMotionSlot );
    leftElevatorCarriageMotorPID.setSmartMotionAllowedClosedLoopError( Constants.CarriageMotorMotionAllowedClosedLoopError, smartMotionSlot);
    leftElevatorCarriageMotorPID.setReference(Constants.CarriageInitPosition, ControlType.kPosition);
    updatePID();
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
    rightInsideElevatorMotorPID.setSmartMotionMaxVelocity(maxVel_Elevator.getDouble(0.0), smartMotionSlot);
    rightInsideElevatorMotorPID.setSmartMotionMinOutputVelocity(minVel_Elevator.getDouble(0.0), smartMotionSlot);
    rightInsideElevatorMotorPID.setSmartMotionMaxAccel(maxAcc_Elevator.getDouble(0.0), smartMotionSlot);
    rightInsideElevatorMotorPID.setSmartMotionAllowedClosedLoopError( Constants.ElevatorMotorMotionAllowedClosedLoopError, smartMotionSlot);
  
    leftElevatorCarriageMotorPID.setSmartMotionMaxVelocity(maxVel_Carriage.getDouble(0.0), smartMotionSlot);
    leftElevatorCarriageMotorPID.setSmartMotionMinOutputVelocity(minVel_Carriage.getDouble(0.0), smartMotionSlot);
    leftElevatorCarriageMotorPID.setSmartMotionMaxAccel(maxAcc_Carriage.getDouble(0.0), smartMotionSlot);
    leftElevatorCarriageMotorPID.setSmartMotionAllowedClosedLoopError( Constants.CarriageMotorMotionAllowedClosedLoopError, smartMotionSlot);
  }
  

  public void updatePID()
  {
    rightInsideElevatorMotorPID.setP(kP_Elevator.getDouble(Constants.ElevatorMotorPID_kP));
    rightInsideElevatorMotorPID.setI(kI_Elevator.getDouble(Constants.ElevatorMotorPID_kI));
    rightInsideElevatorMotorPID.setD(kD_Elevator.getDouble(Constants.ElevatorMotorPID_kD));
    rightInsideElevatorMotorPID.setIZone(kIz_Elevator.getDouble(Constants.ElevatorMotorPID_kIZone));
    rightInsideElevatorMotorPID.setFF(kFF_Elevator.getDouble(Constants.ElevatorMotorPID_kFF));
    rightInsideElevatorMotorPID.setOutputRange(kMinOutput_Elevator.getDouble(-1.0), kMaxOutput_Elevator.getDouble(1.0));
  
    leftElevatorCarriageMotorPID.setP(kP_Carriage.getDouble(Constants.CarriageMotorPID_kP));
    leftElevatorCarriageMotorPID.setI(kI_Carriage.getDouble(Constants.CarriageMotorPID_kI));
    leftElevatorCarriageMotorPID.setD(kD_Carriage.getDouble(Constants.CarriageMotorPID_kD));
    leftElevatorCarriageMotorPID.setIZone(kIz_Carriage.getDouble(Constants.CarriageMotorPID_kIZone));
    leftElevatorCarriageMotorPID.setFF(kFF_Carriage.getDouble(Constants.CarriageMotorPID_kFF));
    leftElevatorCarriageMotorPID.setOutputRange(kMinOutput_Carriage.getDouble(-1.0), kMaxOutput_Carriage.getDouble(1.0));
  }
  
  
  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new ElevatorBottomCMD());
  }
  
   public void elevatorRun()
  {
      setPoint_Elevator = newSetPoint_Elevator.getDouble(0.0);
      elevatorEncoderPos();
      rightInsideElevatorMotorPID.setReference(setPoint_Elevator, ControlType.kPosition);
  } 

  public void elevatorRun(double setPoint)
  {
      setPoint_Elevator = setPoint;
      newSetPoint_Elevator.setDouble(setPoint_Elevator);
      rightInsideElevatorMotorPID.setReference(setPoint_Elevator, ControlType.kPosition);
  } 

  public void carriageRun()
  {
      setPoint_Carriage = newSetPoint_Carriage.getDouble(0.0);
      elevatorEncoderPos();
      leftElevatorCarriageMotorPID.setReference(setPoint_Carriage, ControlType.kPosition);

      // leftElevatorCarriageMotorPID.setReference(setPoint_Carriage, ControlType.kPosition);
  } 

  public void carriageRun(double setPoint)
  {
      setPoint_Carriage = setPoint;
      newSetPoint_Carriage.setDouble(setPoint_Carriage);
      elevatorEncoderPos();
      leftElevatorCarriageMotorPID.setReference(setPoint_Carriage, ControlType.kPosition);
  }

  public void elevatorBottomRun() {
    elevatorEncoderPos();
    rightInsideElevatorMotorPID.setReference(0.15, ControlType.kVoltage);
  }

  public void carriageBottomRun()
  {
      elevatorEncoderPos();
      leftElevatorCarriageMotorPID.setReference(-0.05, ControlType.kVoltage);
  }

  public void elevatorEncoderPos()
  {
    elevatorEncoderPos.setNumber(ElevatorMotorEncoder.getPosition());
    carriageEncoderPos.setNumber(CarriageMotorEncoder.getPosition());
  }

  public boolean elevatorIsMoving()
  {
    return Math.abs(ElevatorMotorEncoder.getPosition() - newSetPoint_Elevator.getDouble(0.0) ) > closeEnough; 
  }


  public boolean carriageIsMoving()
  {
    return Math.abs(CarriageMotorEncoder.getPosition() - newSetPoint_Carriage.getDouble(0.0) ) > closeEnough;
  }

  public boolean elevatorIsUp()
  {
    // true if elevator is higher than the 'UP Limit'. Used to slow the max speed of the bot
    // when the elevator is raised. 
    return ElevatorMotorEncoder.getPosition() > Constants.ElevatorUpLimit;

  }

  public boolean carriageIsUp() {
    return CarriageMotorEncoder.getPosition() > Constants.CarriageUpLimit;
  }
  
  public void stop() {
    ElevatorMotor.stopMotor();
    CarriageMotor.stopMotor();
  } 

  public void updateSmartDashboard(){
    
    SmartDashboard.putNumber("ElevatorCurrent", ElevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("CarriageCurrent", CarriageMotor.getOutputCurrent());
    SmartDashboard.putNumber("ElevatorTemp", ElevatorMotor.getMotorTemperature());
    SmartDashboard.putNumber("CarriageTemp", CarriageMotor.getMotorTemperature());

  }
}



