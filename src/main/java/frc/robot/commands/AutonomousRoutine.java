// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousRoutine extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Routine so the romi can dance :D
   *
   * @param drivetrain The drive subsystem on which this command will run
   */
  public AutonomousRoutine(Drivetrain drivetrain) {
    addCommands(
        new DriveTime(-0.6, 1.5, drivetrain),
        new TurnDegrees(0.67, 180.0, drivetrain),
        new DriveTime(-0.6, 1.6, drivetrain),
        new TurnDegrees(0.67, 180.0, drivetrain),
        new DriveTime(-0.6, 2.0, drivetrain),
        new TurnDegrees(0.7, 40, drivetrain),
        new TurnDegrees(0.7, -80, drivetrain));
  }
}