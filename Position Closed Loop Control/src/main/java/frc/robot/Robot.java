/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private static final int deviceID = 0;
  private CANSparkMax m_motor;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double rotations;

  @Override
  public void robotInit() {
    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor.setParameter(CANSparkMaxLowLevel.ConfigParameter.kF_0, 0);
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.setCANTimeout(20);
    m_motor.setParameter(CANSparkMaxLowLevel.ConfigParameter.kSensorType, 1);
    m_motor.setParameter(CANSparkMaxLowLevel.ConfigParameter.kEncoderSampleDelta, 20);
    // m_motor.setParameter(CANSparkMaxLowLevel.ConfigParameter.kEncoderCountsPerRev, 0.1);
    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();
    

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 0.1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = .1; 
    kMinOutput = -.1;

    // set PID coefficients
    m_pidController.setP(kP, 0);
    m_pidController.setI(kI, 0);
    m_pidController.setD(kD, 0);
    m_pidController.setIZone(kIz, 0);
    m_pidController.setFF(kFF, 0);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput, 0);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

@Override
public void robotPeriodic() {
  super.robotPeriodic();
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    rotations = SmartDashboard.getNumber("Set Rotations", 0);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p, 0); kP = p; }
    if((i != kI)) { m_pidController.setI(i, 0); kI = i; }
    if((d != kD)) { m_pidController.setD(d, 0); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz, 0); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff, 0); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max, 0); 
      kMinOutput = min; kMaxOutput = max; 
    }
}

@Override
public void teleopInit() {
  
  

}

@Override
public void disabledInit() {
  super.disabledInit();
  m_motor.set(0);

}

  @Override
  public void teleopPeriodic() {

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.ControlType.kDutyCycle
     *  com.revrobotics.ControlType.kPosition
     *  com.revrobotics.ControlType.kVelocity
     *  com.revrobotics.ControlType.kVoltage
     */
    m_pidController.setReference(rotations, ControlType.kPosition, 0);
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }
}
