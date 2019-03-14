/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.Position;

public class StowCMD extends Command
{

  public StowCMD() {
    requires(Robot.arm);
  }

  @Override
  protected void initialize()
  {
    Robot.arm.updatePID();
    Robot.arm.updateMotionMagic();
    Robot.arm.goTo(Position.STOW);
  }

  @Override
  protected void execute()
  {
    Robot.arm.updatePosition();
  }

  @Override
  protected boolean isFinished() {
    return Robot.arm.isCloseEnough();
  }

  @Override
  protected void end()
  {
    Robot.arm.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
