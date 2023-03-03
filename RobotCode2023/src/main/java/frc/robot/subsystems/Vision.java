// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.SwerveDrive;



public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  PhotonCamera camera = new PhotonCamera("photonvision");

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

  public void getVisionData() {
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    List<PhotonTrackedTarget> targets = result.getTargets();
    PhotonTrackedTarget bestTarget = result.getBestTarget();

    double yaw = bestTarget.getYaw();
    double pitch = bestTarget.getPitch();
    double area = bestTarget.getArea();
    double skew = bestTarget.getSkew();
    Transform3d pose = bestTarget.getBestCameraToTarget();
    //List<TargetCorner> corners = bestTarget.getCorners();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
