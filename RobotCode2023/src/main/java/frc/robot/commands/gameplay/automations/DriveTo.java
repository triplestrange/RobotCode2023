// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gameplay.automations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;

public class DriveTo extends CommandBase {

  private Pose2d pose2d;
  public double xAutoSpeed = 0;
  public double yAutoSpeed = 0;
  public double rAutoSpeed = 0;
  public double offset;
  public PIDController transformX = new PIDController(2, 0, 0);
  public PIDController transformY = new PIDController(2, 0, 0);
  public PIDController rotation = new PIDController(1.5, 0, 0);
  private SwerveDrive m_SwerveDrive;
  private Robot m_Robot;
  private Pose2d tagPose;
  public Pose2d targetPose;

  /** Creates a new Approach. */
  /*
   * Robot will drive to specified point (parameter)
   */

  public DriveTo(double offset, SwerveDrive m_SwerveDrive, Robot m_Robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SwerveDrive);
    this.m_SwerveDrive = m_SwerveDrive;
    this.m_Robot = m_Robot;
    this.offset = offset;
    rotation.enableContinuousInput(-Math.PI, Math.PI);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ID = m_SwerveDrive.optimalID();
    SmartDashboard.putNumber("ID", ID);
    double finalOffset = ID == 5 || ID == 4 ? Constants.VisionConstants.FEEDER_OFFSET_LEFT.getY()
        : Constants.VisionConstants.CONE_OFFSET_LEFT;
    tagPose = Constants.VisionConstants.tagPose[ID - 1];
    SmartDashboard.putNumber("TAGPOSE X", tagPose.getX());
    SmartDashboard.putNumber("TAGPOSE Y", tagPose.getY());
    SmartDashboard.putNumber("TAGPOSE R", tagPose.getRotation().getDegrees());
    if (m_Robot.allianceColor == Alliance.Blue) {
      tagPose = new Pose2d(tagPose.getX() + 8.27, tagPose.getY() + 4, tagPose.getRotation());
    } else {
      tagPose = new Pose2d(8.27 - tagPose.getX(), 4 - tagPose.getY(),
          tagPose.getRotation().rotateBy(new Rotation2d(Math.PI)));
    }
    SmartDashboard.putNumber("TAGPOSE X updated", tagPose.getX());
    SmartDashboard.putNumber("TAGPOSE Y updated", tagPose.getY());
    SmartDashboard.putNumber("TAGPOSE R updated", tagPose.getRotation().getDegrees());

    targetPose = new Pose2d(tagPose.getX(),
        tagPose.getY() + (finalOffset * offset) - Units.inchesToMeters(1.5) + m_SwerveDrive.getIntakeOffset() / 100,
        tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xAutoSpeed = transformX.calculate(m_SwerveDrive.getPose().getX(), targetPose.getX());
    yAutoSpeed = transformY.calculate(m_SwerveDrive.getPose().getY(), targetPose.getY());
    rAutoSpeed = rotation.calculate(m_SwerveDrive.getAngle().getRadians(), targetPose.getRotation().getRadians());
    // Max Speeds
    xAutoSpeed = MathUtil.clamp(xAutoSpeed, -SwerveConstants.autoAlignMaxSpeedMetersPerSecond,
        SwerveConstants.autoAlignMaxSpeedMetersPerSecond);
    yAutoSpeed = MathUtil.clamp(yAutoSpeed, -SwerveConstants.autoAlignMaxSpeedMetersPerSecond,
        SwerveConstants.autoAlignMaxSpeedMetersPerSecond);
    rAutoSpeed = MathUtil.clamp(rAutoSpeed, -4,
        4);
    SmartDashboard.putNumber("target pose x", targetPose.getX());
    SmartDashboard.putNumber("target pose y", targetPose.getY());
    SmartDashboard.putNumber("target Rotation", targetPose.getRotation().getDegrees());
    if (m_Robot.allianceColor == Alliance.Red) {
      m_SwerveDrive.m_field.getObject("target").setPose(8.27 * 2 - targetPose.getX(), 8 - targetPose.getY(),
          targetPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    } else {
      m_SwerveDrive.m_field.getObject("target").setPose(targetPose);
    }
    m_SwerveDrive.drive(xAutoSpeed, yAutoSpeed, rAutoSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if ((Math.abs(m_SwerveDrive.getPose().getX() - targetPose.getX()) <= 0.05) &&
    // (Math.abs(m_SwerveDrive.getPose().getY() - targetPose.getY()) <= 0.05) &&
    // (Math
    // .abs(m_SwerveDrive.getPose().getRotation().getRadians() -
    // targetPose.getRotation().getRadians()) <= 0.02)) {
    // return true;
    // } else {
    return false;
    // }
  }
}
