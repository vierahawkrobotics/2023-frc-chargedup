// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Needs a joystick for the error to be solved
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoystickArmStateCommand;
import frc.robot.commands.SetArmPosCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;


//import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotContainer {
  ArmSubsystem armSubsystem;
  TelescopeSubsystem telescopeSubsystem;


  public RobotContainer() {
    armSubsystem = new ArmSubsystem();
    telescopeSubsystem = new TelescopeSubsystem();
    configureBindings();
  }
//FYI: you can rebind the buttons on the back on the controller by holding down the middle back button, 
//pressing the button you want to rebind to, and then the button you want to be rebinded
//example: Hold down Back Middle button, press A, press Back Left
  private void configureBindings() {
    ShuffleboardTab tab = Shuffleboard.getTab("TestArmTab");
    
    XboxController joystick = new XboxController(0);
    new Trigger(joystick.povUp(null)).onTrue(new JoystickArmStateCommand(1, armSubsystem, telescopeSubsystem));
    new Trigger(joystick.povDown(null)). onTrue(new JoystickArmStateCommand(-1, armSubsystem, telescopeSubsystem));


    new JoystickButton(joystick, 0).onTrue(new SetArmPosCommand(0.2, armSubsystem));
    new JoystickButton(joystick, 1).onTrue(new SetArmPosCommand(Constants.lowGoalHeight, armSubsystem));
    new JoystickButton(joystick, 2).onTrue(new SetArmPosCommand(Constants.middleGoalHeight, armSubsystem));
    new JoystickButton(joystick, 3).onTrue(new SetArmPosCommand(Constants.highGoalHeight, armSubsystem));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}