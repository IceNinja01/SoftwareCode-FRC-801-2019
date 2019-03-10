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

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The lift subsystem to raise 'em up.
 */
public class Lift extends Subsystem
{

  // A public variable for keeping track of the state of the lift
  public boolean extending = false;

  /* 
  * I forgot what motors I am supposed to be programming for
  * Eh, might as well do them both right?
  */

  // Private motor interface objects
  private CANSparkMax firstMotor, secondMotor;
  private CANPIDController firstPID, secondPID;
  private CANEncoder firstEncoder, secondEncoder;

  // FIXME: Switch/Delete?
  // private TalonSRX firstMotor, secondMotor;

  /*
  * These constants need to be changed to something else.
  * Using as-is will void the warranty on this code.
  */

  // Constants
  private double rotPerInch, travelDist, maxVel, minVel, maxAccel, kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, allowedErr;

  // The function called in the robot start method that initializes this object
  // TODO:  Possibly replace with constructor?
  public void init()
  {

    // Sets a clean state for motor configuration
    firstMotor.restoreFactoryDefaults();
    secondMotor.restoreFactoryDefaults();

    // Grabs the motor encoder and PID objects
    firstEncoder = firstMotor.getEncoder();
    secondEncoder = secondMotor.getEncoder();
    firstPID = firstMotor.getPIDController();
    secondPID = secondMotor.getPIDController();

    /*
    * Initialize constants to real values
    *
    * FIXME: These constants need to be changed to something else.
    *       Using as-is will void the warranty on this code.
    */

    rotPerInch = 0.049;  // Conversion factor for determining setpoint for motion profile.
    travelDist = 1;      // Units of inches       TODO: Find actual value
    maxVel = 50;         // Maximum velocity that will be achieved in the motion profile
    maxAccel = 25;       // Maximum acceleration that will be achieved in the motion profile

    // PID constants
    kP = 5e-4;
    kI = 1e-6;
    kD = 0;
    kIz = 0; // Setting this to 0 disables it. We don't need it.
    kFF = 0; // Might be a good idea to set this to something? What happens with MP?
    kMaxOutput = 1;
    kMinOutput = -1;
    maxVel = 2000; // Units of RPM        FIXME: Is this accurate? What about for other motors?
    maxAccel = 1000; // Units of RPM^2    FIXME: Don't know what this value should be

    /*
    * Initializes all of the PID values for each motor.
    */
    firstPID.setP(kP);
    firstPID.setI(kI);
    firstPID.setD(kD);
    firstPID.setIZone(kIz);
    firstPID.setFF(kFF);
    firstPID.setOutputRange(kMinOutput, kMaxOutput);

    secondPID.setP(kP);
    secondPID.setI(kI);
    secondPID.setD(kD);
    secondPID.setIZone(kIz);
    secondPID.setFF(kFF);
    secondPID.setOutputRange(kMinOutput, kMaxOutput);

    /*
    * Initializes both PIDs for smart motion profiling
    */
    int slotID = 0; // Smart motion slot ID (both motors)
    firstPID.setSmartMotionMaxVelocity(maxVel, slotID);
    firstPID.setSmartMotionMaxAccel(maxAccel, slotID);
    firstPID.setSmartMotionMinOutputVelocity(minVel, slotID);
    firstPID.setSmartMotionAllowedClosedLoopError(allowedErr, slotID);

    secondPID.setSmartMotionMaxVelocity(maxVel, slotID);
    secondPID.setSmartMotionMaxAccel(maxAccel, slotID);
    secondPID.setSmartMotionMinOutputVelocity(minVel, slotID);
    secondPID.setSmartMotionAllowedClosedLoopError(allowedErr, slotID);

    /*
    * Display lift motor PID constants for debug purposes
    * Also display lift motion profiling constants
    */
    SmartDashboard.putNumber("Lift P Gain", kP);
    SmartDashboard.putNumber("Lift I Gain", kI);
    SmartDashboard.putNumber("Lift D Gain", kD);
    SmartDashboard.putNumber("Lift I Zone", kIz);
    SmartDashboard.putNumber("Lift Feed Forward", kFF);
    SmartDashboard.putNumber("Lift Max Output", kMaxOutput);
    SmartDashboard.putNumber("Lift Min Output", kMinOutput);
    SmartDashboard.putNumber("Lift Max Velocity", maxVel);
    SmartDashboard.putNumber("Lift Min Velocity", minVel);
    SmartDashboard.putNumber("Lift Max Acceleration", maxAccel);
    SmartDashboard.putNumber("Lift Allowed Error", allowedErr);
    SmartDashboard.putNumber("Lift Set Position", 0);
    SmartDashboard.putNumber("Lift Set Velocity", 0);
  }

  /*
  * Should we keep this in competitions just in case?
  * This function retrieves constants values from SmartDashboard
  * and writes them to the motor controller.
  */
  public void debugRetrieveConstants()
  {
    double p = SmartDashboard.getNumber("Lift P Gain", 0);
    double i = SmartDashboard.getNumber("Lift I Gain", 0);
    double d = SmartDashboard.getNumber("Lift D Gain", 0);
    double iz = SmartDashboard.getNumber("Lift I Zone", 0);
    double ff = SmartDashboard.getNumber("Lift Feed Forward", 0);
    double max = SmartDashboard.getNumber("Lift Max Output", 0);
    double min = SmartDashboard.getNumber("Lift Min Output", 0);
    double maxV = SmartDashboard.getNumber("Lift Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Lift Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Lift Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Lift Allowed Error", 0); // A?

    // Checks to see if the values changed, if they did, write them to the controllers.
    if((p != kP)) { firstPID.setP(p); secondPID.setP(p); kP = p; }
    if((i != kI)) { firstPID.setI(i); secondPID.setP(p); kI = i; }
    if((d != kD)) { firstPID.setD(d); secondPID.setP(p); kD = d; }
    if((iz != kIz)) { firstPID.setIZone(iz); secondPID.setP(p); kIz = iz; }
    if((ff != kFF)) { firstPID.setFF(ff); secondPID.setP(p); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      firstPID.setOutputRange(min, max);
      secondPID.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel))    { firstPID.setSmartMotionMaxVelocity(maxV,0); 
                              secondPID.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel))    { firstPID.setSmartMotionMaxVelocity(minV,0);
                              secondPID.setSmartMotionMaxVelocity(minV,0); minVel = minV; }
    if((maxA != maxAccel))  { firstPID.setSmartMotionMaxAccel(maxA,0);
                              secondPID.setSmartMotionMaxAccel(maxA,0); maxAccel = maxA; }
    if((allE != allowedErr))
    {
      firstPID.setSmartMotionAllowedClosedLoopError(allE,0);
      secondPID.setSmartMotionAllowedClosedLoopError(allE,0);
      allE = allowedErr;
    }
  }

  public void toggle()
  {
    /*
    * Use of debugRetrieveConstants. See comment on method.
    */
    debugRetrieveConstants();


    //double setPoint, currentPos1, currentPos2;
    double setPoint, currentPos1;

    if (extending) // Assuming that our default state is retract(ed/ing)
    {
      setPoint = rotPerInch * travelDist; // Converts inches to rotations
    }
    else
    {
      setPoint = 0; // Negative or 0 for where we started?
    }

    /**
     * Smart Motion is set by calling the setReference method on an
     * existing pid object and setting the control type to kSmartMotion
     */
    firstPID.setReference(setPoint, ControlType.kSmartMotion);
    secondPID.setReference(setPoint, ControlType.kSmartMotion);

    currentPos1 = firstEncoder.getPosition();
    //currentPos2 = secondEncoder.getPosition();
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("CurrentPos", currentPos1);
    SmartDashboard.putNumber("Output", firstMotor.getAppliedOutput());
  }

  /*
  * This method initializes a default command for the subsystem, which this
  * doesn't need, so we can just leave the method blank.
  */
  @Override
  protected void initDefaultCommand() {}
}
