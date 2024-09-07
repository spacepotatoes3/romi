// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterCm = 2.75591; // 70 mm
  private static final double kWheelDiameterMeter = kWheelDiameterCm / 100.0;

  private final DifferentialDriveKinematics m_kinematics = 
    new DifferentialDriveKinematics(Units.inchesToMeters(5.5));

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private final Field2d m_fieldApproximation = new Field2d(); //may need to initialize pose

    // declare here for methods access, init in constructor
    DifferentialDrivePoseEstimator m_poseEstimator;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_gyro.reset();
    
    // Set up pose estimator
    /* Here we use DifferentialDrivePoseEstimator so that we can fuse odometry readings. The
    numbers used  below are robot specific, and should be tuned. */
    m_poseEstimator = new DifferentialDrivePoseEstimator(
          m_kinematics,
          new Rotation2d(m_gyro.getAngle()),
          (m_leftEncoder.getDistance()),
          (m_rightEncoder.getDistance()),
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    SmartDashboard.putData("FieldEstimation", m_fieldApproximation);

  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceMeter() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }


  /** Updates the field-relative position. */
  public void updateOdometry() {
    
    m_poseEstimator.update(
        new Rotation2d(m_gyro.getAngle()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    // Publish cameraToObject transformation to networktables --this would normally be handled by
    // the
    // computer vision solution.
    //publishCameraToObject(
    //    m_objectInField, m_robotToCamera, m_cameraToObjectEntry, m_drivetrainSimulator);

    // Compute the robot's field-relative position exclusively from vision measurements.
    //Pose3d visionMeasurement3d =
    //    objectToRobotPose(m_objectInField, m_robotToCamera, m_cameraToObjectEntry);

    // Convert robot pose from Pose3d to Pose2d needed to apply vision measurements.
    //Pose2d visionMeasurement2d = visionMeasurement3d.toPose2d();

    // Apply vision measurements. For simulation purposes only, we don't input a latency delay -- on
    // a real robot, this must be calculated based either on known latency or timestamps.
    //m_poseEstimator.addVisionMeasurement(visionMeasurement2d, Timer.getFPGATimestamp());
     
    m_fieldApproximation.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    
  }



}