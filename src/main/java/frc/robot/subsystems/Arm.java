// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private final Servo m_arm = new Servo(Constants.SERVO);
  private double m_liftPos;

  /** Creates a new RomiArm. */
  public Arm() {
    reset();
  }

  public void reset() {
    m_liftPos = 0.5;

    m_arm.set(m_liftPos);
  }

  public void incrementArm(double delta) {
    /* Spec: https://www.pololu.com/docs/0J76/4
     * Range should be 1000 (raised) - 1900 (lowered) us 
     */
    m_liftPos = saturateLimit(m_liftPos + delta, 0 ,.45); 
    m_arm.set(m_liftPos);
  }

  public double get_armPos() {
    return m_liftPos;
  }

  public double saturateLimit(double val, double l_limit, double u_limit) {
    double outval = val;
    if(val > u_limit) {
      outval =  u_limit;
    } else if (val < l_limit) {
      outval = l_limit;
    }
    return outval;
  }

  public double getArmAngle(){
    return m_arm.getAngle();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}