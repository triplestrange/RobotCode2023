// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {}
  /*
   * Methods:
   * - identify AprilTag check
   * - Calculate robot location based on AprilTag check
   * - Calculate distance to each AprilTag check
   * - get biggest (closest) AprilTag check
   * - identify cone vs cube
   * - identify orientation of cone
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
