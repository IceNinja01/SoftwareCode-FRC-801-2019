/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.UpdateSD;

public class SDUpdater extends Subsystem
{
	public boolean isTele;
	
    public void initDefaultCommand()
    {
    	setDefaultCommand(new UpdateSD());
    }

}
