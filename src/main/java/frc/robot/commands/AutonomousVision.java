// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousVision extends SequentialCommandGroup {
  /**
   * Creates a new autonomous routine to run when a target is detected
   * Spins around 180 degrees when a target is detected
   * @param drivetrain The drive subsystem on which this command will run
   */

  public AutonomousVision(Drivetrain drivetrain) {
        addCommands(
        new TurnDegrees(-0.5, 180, drivetrain));
}
}