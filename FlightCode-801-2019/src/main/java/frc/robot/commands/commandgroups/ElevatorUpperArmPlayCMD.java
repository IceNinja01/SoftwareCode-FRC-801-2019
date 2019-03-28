/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Arm.PlayCMD;
import frc.robot.commands.Elevator.ElevatorBottomCMD;
import frc.robot.commands.Elevator.ElevatorLowerPortCMD;
import frc.robot.commands.Elevator.ElevatorMiddlePortCMD;
import frc.robot.commands.Elevator.ElevatorUpperPortCMD;

public class ElevatorUpperArmPlayCMD extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorUpperArmPlayCMD() {

    addParallel(new PlayCMD());
    addSequential(new ElevatorUpperPortCMD());
    
  }
}
