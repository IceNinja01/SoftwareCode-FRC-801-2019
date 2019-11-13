/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utilities.Utils;
import frc.robot.commands.Arm.ArmHoldCMD;
import frc.robot.commands.Arm.ArmManualPositionCMD;
import frc.robot.commands.Arm.ArmSetFudger;
import frc.robot.commands.Arm.ArmStopCMD;

public class Arm extends Subsystem
{

  private TalonSRX armMotor;
  
  private  double kFF = 0.5;
  private  double kP = 1.0;
  private  double kI = 0;
  private  double kD = 0.0;

  public static final int kTimeoutMs = 10;

  public static final double kErrorTolerance = 2;   // Degrees
  
  public static final int kMotorToOutput = 1;      // The ratio of motor turns to output turns.
  public static final int kEncoderTicks = 4096;     // Encoder ticks per rotation of the motor.
  public static final double kMaxVelocity = 1;      // One rotation per second
  public static final double kMaxAcceleration = 1;  // One rotation per second per second
  
  public static final int kPlayPos = 2600;       // Ticks. A Button
  public static final int kDiskPlacePos = 2700;  // Ticks. Y Button
  public static final int kStowPos = 1370;     // Ticks. X Button
  public static final int kBallPos = 1975;       // Ticks. B Button
  public static final int kGather = 2915;

  public static final int kDebugMotorTurn = 48/42; // The test stand has a 6 times gear ratio

  private NetworkTableEntry kP_Arm;
  private NetworkTableEntry kI_Arm;
  private NetworkTableEntry kD_Arm;
  private NetworkTableEntry kFF_Arm;

  private NetworkTableEntry maxVel_Arm;
  private NetworkTableEntry maxAcc_Arm;

  private NetworkTableEntry armEncoderPos;
  private NetworkTableEntry setPoint_Arm;

  // FIXME: Need to actually fix the arm
  private NetworkTableEntry setPoint_Fudger;
  private int fudgeValue_Arm;

  private String ArmTabTitle = "Arm";
  private ShuffleboardTab tab = Shuffleboard.getTab(ArmTabTitle);

  //How much smoothing [0,8] to use during MotionMagic
  int _smoothing = 2;

  private int targetPosition;
  
  public enum Position
  {
    PLAY, DISKPICK, STOW, BALL, GATHER
  }

  public void init()
  {
    armMotor  = new TalonSRX(Constants.ArmPositionMotorID);

    armMotor.configFactoryDefault();

    armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
    armMotor.configReverseSoftLimitThreshold((int) (kEncoderTicks*kDebugMotorTurn*(kStowPos/360)), kTimeoutMs);
    armMotor.configForwardSoftLimitThreshold((int)( kEncoderTicks*kDebugMotorTurn ), kTimeoutMs); // FIXME debug


    // FIXME: Test if this is the correct thing for how it should work
		armMotor.setSensorPhase(false);
		armMotor.setInverted(false);

		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
		
		armMotor.configNominalOutputForward(0, kTimeoutMs);
		armMotor.configNominalOutputReverse(0, kTimeoutMs);
		armMotor.configPeakOutputForward(0.3, kTimeoutMs);
		armMotor.configPeakOutputReverse(-0.3, kTimeoutMs);

		armMotor.selectProfileSlot(0, 0);
		armMotor.config_kF(0, kFF, kTimeoutMs);
		armMotor.config_kP(0, kP, kTimeoutMs);
		armMotor.config_kI(0, kI, kTimeoutMs);
		armMotor.config_kD(0, kD, kTimeoutMs);
    armMotor.configAllowableClosedloopError(0, 10, kTimeoutMs);
		armMotor.configMotionCruiseVelocity((int)(kEncoderTicks*kDebugMotorTurn*kMotorToOutput*(kMaxVelocity/10)), kTimeoutMs); // FIXME debug
    armMotor.configMotionAcceleration((int)(kEncoderTicks*kDebugMotorTurn*kMotorToOutput*(kMaxAcceleration/10)), kTimeoutMs); // FIXME debug
    
    armMotor.setSelectedSensorPosition(armMotor.getSensorCollection().getPulseWidthPosition(), 0, kTimeoutMs);
    
    goTo(Position.DISKPICK);

    initShuffleboard();
    updatePID();
    updateMotionMagic();
  }

  public void initShuffleboard()
  {
    // PID Constants Area
    ShuffleboardLayout ArmMotorPID = Shuffleboard.getTab(ArmTabTitle)
    .getLayout("ArmMotorPID", BuiltInLayouts.kList)
    .withSize(2, 5)
    .withPosition(0, 0);
    kP_Arm = ArmMotorPID.add("kP_Arm", kP).withPosition(0, 0).getEntry();
    kI_Arm = ArmMotorPID.add("kI_Arm", kI).withPosition(0, 1).getEntry();
    kD_Arm = ArmMotorPID.add("kD_Arm", kD).withPosition(0, 2).getEntry();
    kFF_Arm = ArmMotorPID.add("kFF_Arm", kFF).withPosition(0, 3).getEntry();

    // Motion Magic Constants Area
    ShuffleboardLayout ArmMotorMP = Shuffleboard.getTab(ArmTabTitle)
    .getLayout("ArmMotorMP", BuiltInLayouts.kList)
    .withSize(2, 5)
    .withPosition(2, 0);
    maxVel_Arm = ArmMotorMP.add("maxVel_Arm", kMaxVelocity).withPosition(0, 0).getEntry();
    maxAcc_Arm = ArmMotorMP.add("maxAcc_Arm", kMaxAcceleration).withPosition(0, 1).getEntry();
    setPoint_Arm = ArmMotorMP.add("ArmSetPos", 0).withPosition(0, 2).getEntry();
    armEncoderPos = ArmMotorMP.add("ArmGetPos", 0).withPosition(0, 3).getEntry();

    ArmMotorMP.add("SendNewPosition", new ArmManualPositionCMD()).withPosition(0, 4);

    setPoint_Fudger = ArmMotorMP.add("ArmPosFudger", 0).withPosition(0, 5).getEntry();

    ArmMotorMP.add("SendNewFudger", new ArmSetFudger()).withPosition(0, 6);
  }

  public void updateFudger()
  {
    fudgeValue_Arm = setPoint_Fudger.getNumber(0).intValue();
  }

  public void updatePID()
  {
    armMotor.config_kF(0, kFF_Arm.getDouble(kFF), kTimeoutMs);
		armMotor.config_kP(0, kP_Arm.getDouble(kP), kTimeoutMs);
		armMotor.config_kI(0, kI_Arm.getDouble(kI), kTimeoutMs);
		armMotor.config_kD(0, kD_Arm.getDouble(kD), kTimeoutMs);
  }

  public void updateMotionMagic()
  {
    armMotor.configMotionCruiseVelocity((int)(kEncoderTicks*kDebugMotorTurn*kMotorToOutput*(maxVel_Arm.getDouble(kMaxVelocity)/10)), kTimeoutMs); // FIXME debug
    armMotor.configMotionAcceleration((int)(kEncoderTicks*kDebugMotorTurn*kMotorToOutput*(maxAcc_Arm.getDouble(kMaxAcceleration)/10)), kTimeoutMs); // FIXME debug
  }

  public void updatePosition()
  {
    armEncoderPos.setNumber(getCurrentPosition());
  }

  public void stop()
  {
    armMotor.set(ControlMode.PercentOutput, 0.0);
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void goTo(Position pos)
  {
        armMotor.setNeutralMode(NeutralMode.Coast);

    targetPosition = 0;

    switch (pos)
    {
      case PLAY:
      targetPosition = kPlayPos;
        break;
      case DISKPICK:
      targetPosition = kDiskPlacePos;
        break;
      case STOW:
      targetPosition = kStowPos;
        break;
      case BALL:
      targetPosition = kBallPos;
        break;
      case GATHER:
      targetPosition = kGather;
      break;
    }

    targetPosition += fudgeValue_Arm;

    armMotor.set(ControlMode.MotionMagic, targetPosition);
  }

  public void goTo(double ticks)
  {
    armMotor.setNeutralMode(NeutralMode.Coast);
    // int setPoint = (int) Utils.wrapAngle0To360Deg(degrees - Constants.ArmAngleBias);
    // setPoint *= kEncoderTicks*kDebugMotorTurn;
    // setPoint /= 360;
    int setPoint = (int) ticks;
    targetPosition = setPoint;
    SmartDashboard.putNumber("Target Postion", targetPosition);

    SmartDashboard.putNumber("armCMDSetpoint", setPoint);
    armMotor.set(ControlMode.MotionMagic, setPoint);
  }

  public void goToManual()
  {
    double setPoint = setPoint_Arm.getDouble(0.0);
    updatePosition();
    goTo(setPoint);
  }

  public double getCurrentPosition()
  {
    int absolutePosition = armMotor.getSensorCollection().getPulseWidthPosition();
    int position = armMotor.getSelectedSensorPosition();
    // double positionDegrees = Utils.wrapAngle0To360Deg(position);
    SmartDashboard.putNumber("ArmRelativePos", position);
    double positionDegrees = (position)*(360.0/4096.0);
    //positionDegrees = positionDegrees - Constants.ArmAngleBias;
    positionDegrees = Utils.wrapAngle0To360Deg(positionDegrees);
    armEncoderPos.setNumber(positionDegrees);
    SmartDashboard.putNumber("ArmEncoderAbs", absolutePosition);
    return absolutePosition;
  }

  public double getCurrentError()
  {
    double position = getCurrentPosition();
    double error = targetPosition - position;

    return Math.abs(error);
  }

  public boolean isCloseEnough()
  {
    return getCurrentError() < kErrorTolerance;
  }

  @Override
  public void initDefaultCommand()
  {
    setDefaultCommand(new ArmStopCMD());
  }

public void updateSD() {

    SmartDashboard.putNumber("ArmEncoder", getCurrentPosition());
}

  
}