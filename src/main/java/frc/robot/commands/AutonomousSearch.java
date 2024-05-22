// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousSearch extends SequentialCommandGroup {
  /**
   * Creates a new autonomous routine to run while searching for a target
   * Turns 360 degrees to search for the target
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousSearch(Drivetrain drivetrain) {
    addCommands(
        new TurnDegrees(-0.4, 90, drivetrain));
  }
}
