// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.logging.Logger;
import java.util.logging.Level;

public class VisionAutonomous extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on time. This will drive out for a period of time, turn
   * around for time (equivalent to time to turn around) and drive forward again. This should mimic
   * driving out, turning around and driving back.
   *
   * @param drivetrain The drive subsystem on which this command will run
   */
  NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
  NetworkTable table = ntinst.getTable("Vision");
  
  NetworkTableEntry tagEntry = table.getEntry("TagDetected");
  boolean tagDetected = tagEntry.getBoolean(false);

  
  

  public VisionAutonomous(Drivetrain drivetrain) {

    printMessage();
   NetworkTableEntry activeEntry = table.getEntry("Active");
    activeEntry.setBoolean(true);

    long tagID = table.getEntry("tagID").getInteger(0);

    while (tagID == 0) {
      //wait until tag is detected
    }
          
          NetworkTableEntry visionEntry = table.getEntry("Running Vision");
          visionEntry.setBoolean(true);
    addCommands(
        new DriveDistance(-0.7, 10, drivetrain),
        new DriveDistance(0.7, 10, drivetrain));
    
  }

  public static void printMessage() {
    Logger logger = Logger.getLogger("MyLogger");
    logger.info("VisionAutonomos is running");
  }

}
