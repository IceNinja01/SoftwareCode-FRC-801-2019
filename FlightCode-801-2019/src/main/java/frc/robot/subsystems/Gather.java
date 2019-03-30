/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.Gather.GatherStopCMD;

/**
 * Add your docs here.
 */
public class Gather extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  //private CANSparkMax rightLiftMotor;
  //private CANEncoder rightLiftEncoder;
  //private CANPIDController rightLiftPID; 

  private TalonSRX gatherMotorUpper;
  private int kTimeoutMs = 10;
  private TalonSRX gatherMotorLower;

  public void init() {
    gatherMotorUpper  = new TalonSRX(Constants.GatherMotorUpperID);
    gatherMotorLower  = new TalonSRX(Constants.GatherMotorLowerID);

    gatherMotorUpper.configFactoryDefault();
    gatherMotorLower.configFactoryDefault();
    
		gatherMotorUpper.configNominalOutputForward(0, kTimeoutMs);
		gatherMotorUpper.configNominalOutputReverse(0, kTimeoutMs);
		gatherMotorUpper.configPeakOutputForward(1, kTimeoutMs);
    gatherMotorUpper.configPeakOutputReverse(-1, kTimeoutMs);

    gatherMotorLower.configNominalOutputForward(0, kTimeoutMs);
		gatherMotorLower.configNominalOutputReverse(0, kTimeoutMs);
		gatherMotorLower.configPeakOutputForward(1, kTimeoutMs);
    gatherMotorLower.configPeakOutputReverse(-1, kTimeoutMs);

    gatherMotorLower.setInverted(true);
    gatherMotorUpper.setInverted(false);

  }

  @Override
  public void initDefaultCommand() {
   setDefaultCommand(new GatherStopCMD());
  }

  public void stop() {
    gatherMotorLower.set(ControlMode.PercentOutput, 0.0);
    gatherMotorUpper.set(ControlMode.PercentOutput, 0.0);
    gatherMotorLower.setNeutralMode(NeutralMode.Brake);
    gatherMotorUpper.setNeutralMode(NeutralMode.Brake);

  }

  public void gatherBall(){
    gatherMotorLower.set(ControlMode.PercentOutput, 0.5);
    gatherMotorUpper.set(ControlMode.PercentOutput, 0.5);

  }

  public void ejectBall(){
    gatherMotorLower.set(ControlMode.PercentOutput, -1.0);
    gatherMotorUpper.set(ControlMode.PercentOutput, -1.0);

  }
  
}
