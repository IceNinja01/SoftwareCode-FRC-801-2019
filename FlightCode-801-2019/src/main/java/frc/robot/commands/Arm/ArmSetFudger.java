/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// https://prod.liveshare.vsengsaas.visualstudio.com/join?36E6AEEB7F6F46B47801D0114B8090F33994

package frc.robot.commands.Arm; // dwawda

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ArmSetFudger extends Command
{
 
  public ArmSetFudger() {
    requires(Robot.arm);
  }

  @Override
  protected void initialize()
  {
    Robot.arm.updateFudger();
  }

  @Override
  protected void execute()
  {
    // Nothing
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end()
  {
    // Nothing
  }

  @Override
  protected void interrupted() {
    end();
  }
}
