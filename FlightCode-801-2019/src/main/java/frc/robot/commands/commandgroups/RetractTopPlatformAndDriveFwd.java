/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Chassis.DriveFwd;
import frc.robot.commands.Lift.LiftRetractTopPositionCMD;

public class RetractTopPlatformAndDriveFwd extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RetractTopPlatformAndDriveFwd() {
    requires(Robot.chassis);
    requires(Robot.lift);
    addParallel( new LiftRetractTopPositionCMD());
    addSequential( new  DriveFwd(0.0, 0.2, 6.0));
  }
}
