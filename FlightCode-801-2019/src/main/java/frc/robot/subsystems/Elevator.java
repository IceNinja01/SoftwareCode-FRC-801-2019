/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
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
      rightInsideElevatorMotorPID.setP(kP);
      rightInsideElevatorMotorPID.setI(kI);
      rightInsideElevatorMotorPID.setD(kD);
      rightInsideElevatorMotorPID.setIZone(kIz);
      rightInsideElevatorMotorPID.setFF(kFF);
      rightInsideElevatorMotorPID.setOutputRange(kMinOutput, kMaxOutput);
  
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
      rightInsideElevatorMotorPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      rightInsideElevatorMotorPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
      rightInsideElevatorMotorPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      rightInsideElevatorMotorPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", kFF);
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);
  
      // display Smart Motion coefficients
      SmartDashboard.putNumber("Max Velocity", maxVel);
      SmartDashboard.putNumber("Min Velocity", minVel);
      SmartDashboard.putNumber("Max Acceleration", maxAcc);
      SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
      SmartDashboard.putNumber("Set Position", 0);
      SmartDashboard.putNumber("Set Velocity", 0);
  
      // button to toggle between velocity and smart motion modes
      SmartDashboard.putBoolean("Mode", true);
}

public void initDashboard(){
//ElevatorMotor
ShuffleboardLayout ElevatorMotor = Shuffleboard.getTab(ElevatorTitle)
  .getLayout("ElevatorMotor", BuiltInLayouts.kList)
  .withSize(2, 5)
  .withPosition(0, 0);
kP_Elevator = ElevatorMotor.add("kP_Elevator", .0005).getEntry();
kI_Elevator = ElevatorMotor.add("kI_Elevator", 1e-6).getEntry();
kD_Elevator = ElevatorMotor.add("kD_Elevator", 0).getEntry();
kIz_Elevator = ElevatorMotor.add("kIz_Elevator", 0).getEntry(); 
kFF_Elevator = ElevatorMotor.add("kFF_Elevator", 0).getEntry(); 
kMaxOutput_Elevator = ElevatorMotor.add("kMaxOutput_Elevator", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();

//CarriageMotor
ShuffleboardLayout CarriageMotor = Shuffleboard.getTab(ElevatorTitle)
 .getLayout("CarriageMotor", BuiltInLayouts.kList)
 .withSize(2, 5)
 .withPosition(2, 0);
kP_Carriage = CarriageMotor.add("kP_Carriage", 0.01).getEntry();
kI_Carriage = CarriageMotor.add("kI_Carriage", 0.01).getEntry();
kD_Carriage = CarriageMotor.add("kD_Carriage", 0).getEntry();
kIz_Carriage = CarriageMotor.add("kIz_Carriage", 0).getEntry(); 
kFF_Carriage = CarriageMotor.add("kFF_Carriage", 0.0001).getEntry(); 
kMaxOutput_Carriage = CarriageMotor.add("kMaxOutput_Carriage", 1.0).getEntry(); 

}

public void elevMove()
{
  // read PID coefficients from SmartDashboard
  double p = SmartDashboard.getNumber("P Gain", 0);
  double i = SmartDashboard.getNumber("I Gain", 0);
  double d = SmartDashboard.getNumber("D Gain", 0);
  double iz = SmartDashboard.getNumber("I Zone", 0);
  double ff = SmartDashboard.getNumber("Feed Forward", 0);
  double max = SmartDashboard.getNumber("Max Output", 0);
  double min = SmartDashboard.getNumber("Min Output", 0);
  double maxV = SmartDashboard.getNumber("Max Velocity", 0);
  double minV = SmartDashboard.getNumber("Min Velocity", 0);
  double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
  double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != kP)) { rightInsideElevatorMotorPID.setP(p); kP = p; }
  if((i != kI)) { rightInsideElevatorMotorPID.setI(i); kI = i; }
  if((d != kD)) { rightInsideElevatorMotorPID.setD(d); kD = d; }
  if((iz != kIz)) { rightInsideElevatorMotorPID.setIZone(iz); kIz = iz; }
  if((ff != kFF)) { rightInsideElevatorMotorPID.setFF(ff); kFF = ff; }
  if((max != kMaxOutput) || (min != kMinOutput)) { 
    rightInsideElevatorMotorPID.setOutputRange(min, max); 
    kMinOutput = min; kMaxOutput = max; 
  }
  if((maxV != maxVel)) { rightInsideElevatorMotorPID.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
  if((minV != minVel)) { rightInsideElevatorMotorPID.setSmartMotionMaxVelocity(minV,0); minVel = minV; }
  if((maxA != maxAcc)) { rightInsideElevatorMotorPID.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
  if((allE != allowedErr)) { rightInsideElevatorMotorPID.setSmartMotionAllowedClosedLoopError(allE,0); allE = allowedErr; }

  double setPoint, processVariable;
  boolean mode = SmartDashboard.getBoolean("Mode", false);
  if(mode) {
    setPoint = SmartDashboard.getNumber("Set Velocity", 0);
    rightInsideElevatorMotorPID.setReference(setPoint, ControlType.kVelocity);
    processVariable = rightInsideElevatorMotorEncoder.getVelocity();
  } else {
    setPoint = SmartDashboard.getNumber("Set Position", 0);
    /**
     * As with other PID modes, Smart Motion is set by calling the
     * setReference method on an existing pid object and setting
     * the control type to kSmartMotion/''
     */
    rightInsideElevatorMotorPID.setReference(setPoint, ControlType.kSmartMotion);
    processVariable = rightInsideElevatorMotorEncoder.getPosition();
  }
  
  SmartDashboard.putNumber("SetPoint", setPoint);
  SmartDashboard.putNumber("Process Variable", processVariable);
  SmartDashboard.putNumber("Output", rightInsideElevatorMotor.getAppliedOutput());
}

  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
