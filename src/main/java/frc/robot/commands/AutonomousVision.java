// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.logging.Logger;

public class AutonomousVision extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on time. This will drive out for a period of time, turn
   * around for time (equivalent to time to turn around) and drive forward again. This should mimic
   * driving out, turning around and driving back.
   *
   * @param drivetrain The drive subsystem on which this command will run
   */
  static NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
  static NetworkTable table = ntinst.getTable("Vision");

  NetworkTableEntry tagEntry = table.getEntry("TagDetected");
  boolean tagDetected = tagEntry.getBoolean(false);

  public AutonomousVision(Drivetrain drivetrain) {
        System.out.println("AutonomousVision running");
        addCommands(
        new DriveDistance(-0.4, 10, drivetrain),
        new DriveDistance(0.4, 10, drivetrain));
}


  public void run(Drivetrain drivetrain) {
       
    printMessage();
   NetworkTableEntry activeEntry = table.getEntry("Active");
    activeEntry.setBoolean(true);

          NetworkTableEntry visionEntry = table.getEntry("Running Vision");
          visionEntry.setBoolean(true);
  }
  public static void printMessage() {
    Logger logger = Logger.getLogger("MyLogger");
    logger.info("VisionAutonomous is running");
  }

  public static String commandToString(Command command) {
    if (command != null) {
    return command.getClass().getName();
    } else {
        return "";
    }

  }
}