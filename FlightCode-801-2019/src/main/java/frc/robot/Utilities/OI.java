/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utilities;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.Utilities.XBOXJoystick;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Gather.*;
import frc.robot.commands.Lift.*;
import frc.robot.commands.Pincher.RetractCMD;
import frc.robot.commands.Pincher.StabCMD;
import frc.robot.commands.commandgroups.RetractTopPlatformAndDriveFwd;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public XBOXJoystick driver = new XBOXJoystick(0);
  public XBOXJoystick manipulator = new XBOXJoystick(1);

  //
    //	      5	    _                            _    6
    //	       _.-'` `-._                    _,-' `'-._
    //	    ,-'          `-.,____________,.-'    .-.   `-.
    //	   /   .---.             ___            ( 4 )     \
    //	  /  ,' ,-. `.     __   / X \   __   .-. `-` .-.   \
    //	 /   | | 9 | |    (_7) | / \ | (_8) ( 3 )   ( 2 )   \
    //	/    `. `-' ,'    __    \___/        `-` ,-. `-`     \
    //	|      `---`   ,-`  `-.       .---.     ( 1 )        |
    //	|             / -'  `- \    ,'  .  `.    `-`         |
    //	|            |          |   | -10 - |                |
    //	!             \ -.  ,- /    `.  '  ,'                |
    //	|              `-.__,-'       `---`                  |
    //	|                  ________________                  |
    //	|             _,-'`                ``-._             |
    //	|          ,-'                          `-.          |
    //	 \       ,'                                `.       /
    //	  `.__,-'                                    `-.__,'
    //
   
    // Elevator mapping
    Button elevatorBottomButton = new JoystickButton(manipulator, 1);     // 'A' button
    Button elevatorLowerPortButton = new JoystickButton(manipulator, 3);  // 'X' button
    Button elevatorMiddlePortButton = new JoystickButton(manipulator, 2); // 'B' button
    Button elevatorUpperPortButton = new JoystickButton(manipulator, 4);  // 'Y' button
    Button elevatorGatherButton = new JoystickButton(manipulator, 6);     // Right Top Bumper

    // Lift Mapping
    Button liftTopPlatformButton = new JoystickButton(driver, 8);       // Right Bottom bumper
    Button retractAndDriveButton = new JoystickButton(driver, 7);   // Left Bottom bumper

    // Gather Mapping
    Button gatherButton = new JoystickButton(driver, 5);    // Left bumper
    Button ejectButton = new JoystickButton(driver, 5);     // Right bumper
    Button stabButton = new POVButton(driver, 0);           // POV UP
    Button retractButton = new POVButton(driver, 180);      // POV Down

    // Arm mapping
    Button playButton = new JoystickButton(driver, 1);      // 'A' button
    Button diskPickButton = new JoystickButton(driver, 4);  // 'Y' button
    Button stowButton = new JoystickButton(driver, 3);      // 'X' button
    Button ballButton = new JoystickButton(driver, 2);      // 'B' button
    
  public OI() {

    // elevator mapping
    elevatorBottomButton.whenPressed( new ElevatorBottomCMD() );
    elevatorLowerPortButton.whenPressed( new ElevatorLowerPortCMD() );
    elevatorMiddlePortButton.whenPressed( new ElevatorMiddlePortCMD() );
    elevatorUpperPortButton.whenPressed( new ElevatorUpperPortCMD() );
    elevatorGatherButton.whenPressed( new ElevatorGatherCMD() );
    
    // Lift Mapping
    liftTopPlatformButton.whenPressed( new LiftTopPlatformCMD() );
    retractAndDriveButton.whenPressed( new RetractTopPlatformAndDriveFwd() );
    
    // Gather Mapping
    gatherButton.whileHeld( new GatherCMD() );
    ejectButton.whileHeld( new EjectCMD() );
    stabButton.whenPressed( new StabCMD() );
    retractButton.whenPressed( new RetractCMD() );
    
    // Arm mapping
    playButton.whenPressed( new PlayCMD() );
    diskPickButton.whenPressed( new DiskPickCMD() );
    stowButton.whenPressed( new StowCMD() );
    ballButton.whenPressed( new BallCMD() );

  }
}
