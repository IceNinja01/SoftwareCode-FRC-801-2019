/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.Pincher.RetractCMD;
import frc.robot.commands.Pincher.StabCMD;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//Used for the hatch graber
public class Pincher extends Subsystem {
	
	public static DoubleSolenoid pincher;

	public Pincher() {

	}

	public void init() {
		pincher = new DoubleSolenoid(21,0,1); 
		
	}
	
	protected void initDefaultCommand() {
		setDefaultCommand(new StabCMD());
	}
	////Pinchers code below
	public void closePinchers() {
		
		pincher.set(DoubleSolenoid.Value.kReverse);

	}
	
	public void openPinchers() {
		
		pincher.set(DoubleSolenoid.Value.kForward);
	
	}	

}