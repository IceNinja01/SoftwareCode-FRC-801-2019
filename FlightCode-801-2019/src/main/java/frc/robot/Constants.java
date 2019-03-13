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
    public static final int[] AngleBias = {1626, 1644, 947, 2001};
	public static final int rightLiftMotorID = 8;
	public static final int leftLiftMotorID = 9;
	
	public static final int rightInsideElevatorMotorID = 10;
	public static final int leftElevatorCarriageMotorID = 11;
	
	public static final int ArmPositionMotorID = 12;
	public static final int GatherMotorUpperID = 13;
	public static final int GatherMotorLowerID = 14;

	public static final int RightFrontSteer = 0;
	public static final int LeftFrontSteer = 1;
	public static final int LeftRearSteer = 2;
	public static final int RightRearSteer = 3;

	public static final int RightFrontDrive = 4;
	public static final int LeftFrontDrive = 5;
	public static final int LeftRearDrive = 6;
	public static final int RightRearDrive = 7;

	public static double rotations_per_deg = 1;


    


}
