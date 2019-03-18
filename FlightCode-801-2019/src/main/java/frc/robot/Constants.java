/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {

	/*Work on using preferences tables
	public static double FrontRightBias = Robot.prefs.getDouble("FrontRightBias", 0.0);
	public static double FrontLeftBias = Robot.prefs.getDouble("FrontLeftBias", 0.0);
	public static double BackLeftBias = Robot.prefs.getDouble("BackLeftBias", 0.0);
	public static double BackRightBias = Robot.prefs.getDouble("BackRightBias", 0.0);
	*/
	public static final int[] AngleBias = {1502, 2149, 1366, 104};

	public static final int ArmMotorBias = 0; // TODO: Find out bias
	public static double ArmAngleBias = 2776;    //absolute angle bias

	// Motor CAN Bus ID's
	public static final int rightLiftMotorID = 8;
	public static final int leftLiftMotorID = 9;
	
	public static final int rightInsideElevatorMotorID = 10;
	public static final int leftElevatorCarriageMotorID = 11;
	
	public static final int ArmPositionMotorID = 12;
	public static final int GatherMotorUpperID = 0;  // depricated
	public static final int GatherMotorLowerID = 14;

	public static final int RightFrontSteerMotorID = 13;
	public static final int LeftFrontSteerMotorID = 1;
	public static final int LeftRearSteerMotorID = 2;
	public static final int RightRearSteerMotorID = 3;

	public static final int RightFrontDriveMotorID = 4;
	public static final int LeftFrontDriveMotorID = 5;
	public static final int LeftRearDriveMotorID = 6;
	public static final int RightRearDriveMotorID = 7;

	//Swerve Pod PID constants
	public static final double kP_DriveMotors = 0.0001;
	public static final double kI_DriveMotors = 0.0;
	public static final double kD_DriveMotors = 0.0;
	public static final double kIz_DriveMotors = 0.0;
	public static final double kFF_DriveMotors = 0.0001818;
	public static final double kOutputRangeMax_DriveMotors = 1.0; 
	public static final double kOutputRangeMmin_DriveMotors = -1.0; 

	public static final double kP_TurnMotors = 0.005;
	public static final double kI_TurnMotors = 0.0001;
	public static final double kD_TurnMotors = 0.0;
	public static final int kIz_TurnMotors = 0;
	public static final double kFF_TurnMotors = 0.0;
	public static final double kOutputRangeMax_TurnMotors = 1.0; 
	public static final double kOutputRangeMmin_TurnMotors = -1.0;
	public static final int deadBand_TurnMotors = 0;

////====================================////
	public static double rotations_per_deg = 1;

	// Lift positions in INCHES
	public static double LiftTopPlatformPosition = 27.0;
	public static double LiftRetractTopPosition = 10.0;
	public static double LiftMidPlatformPosition = 12.0;
	public static double LiftRetractMidPosition = 9.0;

	// Lift Motor Settings.
	public static double LiftMotorPID_kP = 0.001;
	public static double LiftMotorPID_kI = 0.0;
	public static double LiftMotorPID_kD = 0.0;
	public static double LiftMotorPID_kIZone = 0.0;
	public static double LiftMotorPID_kFF = 0.0;
	public static double LiftMotorPID_kOutputRangeMax = 1.0 ;
	public static double LiftMotorPID_kOutputRangeMin = -1.0;
	
    public static double LiftMotorMotionMaxVelocity = 5700;
	public static double LiftMotorMotionMinOutputVelocity = 0.0;
    public static double LiftMotorMotionMaxAccel = 7500;
    public static double LiftMotorMotionAllowedClosedLoopError = 0.0;

	// Elevator and Carriage positions in INCHES
	public static double ElevatorInitPosition = 0.0;
	public static double ElevatorBottomPosition = 0.0;
	public static double ElevatorGatherPosition = 0.0;
	public static double ElevatorLowerPosition = 0.0;
	public static double ElevatorMiddlePosition = 4.0;
	public static double ElevatorUpperPosition = 32.0;

	public static double ElevatorUpLimit = 15.0;  // This is the height where the robot drive speed limits to 50%

	public static double CarriageInitPosition = 0.0;
	public static double CarriageBottomPosition = 0.0;
	public static double CarriageGatherPosition = 0.0;
	public static double CarriageLowerPosition = 6.0;
	public static double CarriageMiddlePosition = 30.0;
	public static double CarriageUpperPosition = 30.0;
	
	// Elevator Motor Settings.
	public static double ElevatorMotorPID_kP = 0.0005;
	public static double ElevatorMotorPID_kI = 0.0001;
	public static double ElevatorMotorPID_kD = 0.0;
	public static double ElevatorMotorPID_kIZone = 0.0;
	public static double ElevatorMotorPID_kFF = 0.0;
	public static double ElevatorMotorPID_kOutputRangeMax = 1.0 ;
	public static double ElevatorMotorPID_kOutputRangeMin = -1.0;
  
	public static double CarriageMotorPID_kP = 0.0005;
	public static double CarriageMotorPID_kI = 0.0001;
	public static double CarriageMotorPID_kD = 0.0;
	public static double CarriageMotorPID_kIZone = 0.0;
	public static double CarriageMotorPID_kFF = 0.0;
	public static double CarriageMotorPID_kOutputRangeMax = 1.0; 
	public static double CarriageMotorPID_kOutputRangeMmin = -1.0; 

    public static double ElevatorMotorMotionMaxVelocity = 2500;
	public static double ElevatorMotorMotionMinOutputVelocity = 0.0;
    public static double ElevatorMotorMotionMaxAccel = 2500;
    public static double ElevatorMotorMotionAllowedClosedLoopError = 0.0;
  
    public static double CarriageMotorMotionMaxVelocity = 2500;
	public static double CarriageMotorMotionMinOutputVelocity = 0.0;
	public static double CarriageMotorMotionMaxAccel = 2500;
	public static double CarriageMotorMotionAllowedClosedLoopError = 0.0;

}
