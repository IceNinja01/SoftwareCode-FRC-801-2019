/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Pincher;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class StabCMD extends Command {
  public StabCMD() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.pincher);
  }
    
  // Called once when the command executes
  protected void initialize() {  
    Robot.pincher.openPinchers();
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
  
}