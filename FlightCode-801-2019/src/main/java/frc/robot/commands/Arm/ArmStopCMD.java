/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ArmStopCMD extends Command {
  public ArmStopCMD()
  {
    requires(Robot.arm);
  }

  @Override
  protected void initialize()
  {
    Robot.arm.stop();
  }

  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end()
  {
    Robot.arm.stop();
  }

  @Override
  protected void interrupted()
  {
    end();
  }
}
