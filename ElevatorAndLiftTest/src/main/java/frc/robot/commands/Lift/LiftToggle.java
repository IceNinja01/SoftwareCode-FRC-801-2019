/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/*
* Toggles the lift between retracted and extended
*/
public class LiftToggle extends Command
{

  public LiftToggle()
  {
    requires(Robot.lift);
  }

  @Override
  protected void initialize()
  {
    System.out.println("Moving the robot up or down");
    // FIXME: Might have to add some code in here
  }

  @Override
  protected void execute()
  {
    Robot.lift.toggle();
  }

  @Override
  protected boolean isFinished() {
    return false;
    // TODO: Should we do something in here this year?
  }

  @Override
  protected void end() {}

  @Override
  protected void interrupted() {}
}
