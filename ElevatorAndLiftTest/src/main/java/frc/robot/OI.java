/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.Utilities.XBOXJoystick;
import frc.robot.commands.Lift.LiftToggle;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public XBOXJoystick driver = new XBOXJoystick(0);
  public XBOXJoystick manipulator = new XBOXJoystick(1);

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);
   
    // Elevator mapping
    Button elevatorBottomButton = new JoystickButton(manipulator, 1);     // 'A' button
    Button elevatorLowerPortButton = new JoystickButton(manipulator, 3);  // 'X' button
    Button elevatorMiddleButton = new JoystickButton(manipulator, 2);     // 'B' button
    Button elevatorUpperPortButton = new JoystickButton(manipulator, 4);  // 'Y' button
    Button elevatorGatherButton = new JoystickButton(manipulator, 6);     // Right Bumper

    // Lift Mapping
    Button liftUpDownToggleButton = new JoystickButton(driver, 8);        // start button

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
    /*
    elevatorBottomButton.whenPressed(command); // TODO
    elevatorLowerPortButton.whenPressed(command); // TODO
    elevatorMiddleButton.whenPressed(command); // TODO
    elevatorUpperPortButton.whenPressed(command); // TODO
    elevatorGatherButton.whenPressed(command); // TODO
    */
    
    // Lift Mapping
    liftUpDownToggleButton.whenPressed(new LiftToggle());
    
    // Gather Mapping
    /*
    gatherButton.whenPressed(command); // TODO
    ejectButton.whenPressed(command); // TODO
    stabButton.whenPressed(command); // TODO
    retractButton.whenPressed(command); // TODO
    */
    
    // Arm mapping
    /*
    playButton.whenPressed(command); // TODO
    diskPickButton.whenPressed(command); // TODO
    stowButton.whenPressed(command); // TODO
    ballButton.whenPressed(command); // TODO
    */
    

  }
}
