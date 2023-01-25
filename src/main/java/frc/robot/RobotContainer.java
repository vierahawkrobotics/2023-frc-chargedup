// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RotationCommand;
import frc.robot.subsystems.Collector;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  Collector collector;


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    CommandJoystick joystick = new CommandJoystick(0);
    joystick.button(1).whileTrue(new RotationCommand(collector, 1));
    joystick.button(2).whileTrue(new RotationCommand(collector, -1));
    collector.setDefaultCommand(new RotationCommand(collector, 0));

    
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
