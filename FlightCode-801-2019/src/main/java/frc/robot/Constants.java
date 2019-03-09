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
    public static final int[] AngleBias = {0, 0, 0, 0};
	public static final int rightLiftMotorID = 0;
	public static final int RightFrontSteer = 6;
	public static final int LeftFrontSteer = 9;
	public static final int LeftRearSteer = 11;
	public static final int RightRearSteer = 4;

	public static final int RightFrontDrive = 3;
	public static final int LeftFrontDrive = 12;
	public static final int LeftRearDrive = 15;
	public static final int RightRearDrive = 0;


	public static double rotations_per_deg = 1;


    


}
