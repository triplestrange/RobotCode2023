/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Electrical;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Robot;
// import frc.robot.Constants.ModuleConstants;
// import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;

@SuppressWarnings("PMD.ExcessiveImports")
public class SwerveDrive extends SubsystemBase {
  private final Robot m_Robot;
  public PIDController transformX = new PIDController(0.05, 0, 0);
  public PIDController transformY = new PIDController(0.05, 0, 0);
  public PIDController rotation = new PIDController(0.05, 0, 0);
  public double xAutoSpeed = 0;
  public double yAutoSpeed = 0;
  public double rAutoSpeed = 0;
  public double rotationPreset = 0;
  public boolean presetEnabled = false;
  public double tv = 0;
  Pose2d visionPose = new Pose2d();
  double[] tempRobotPose;

  private AnalogInput intakeLeft;
  private AnalogInput intakeRight;

  // Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(Electrical.FL_DRIVE,
      Electrical.FL_STEER,
      ModuleConstants.FL_ENCODER,
      SwerveConstants.frontLeftSteerEncoderReversed,
      ModuleConstants.FL_ENC_OFFSET);

  private final SwerveModule m_rearLeft = new SwerveModule(Electrical.BL_DRIVE,
      Electrical.BL_STEER,
      ModuleConstants.BL_ENCODER,
      SwerveConstants.backLeftSteerEncoderReversed,
      ModuleConstants.BL_ENC_OFFSET);

  private final SwerveModule m_frontRight = new SwerveModule(Electrical.FR_DRIVE,
      Electrical.FR_STEER,
      ModuleConstants.FR_ENCODER,
      SwerveConstants.frontRightSteerEncoderReversed,
      ModuleConstants.FR_ENC_OFFSET);

  private final SwerveModule m_rearRight = new SwerveModule(Electrical.BR_DRIVE,
      Electrical.BR_STEER,
      ModuleConstants.BR_ENCODER,
      SwerveConstants.backRightSteerEncoderReversed,
      ModuleConstants.BR_ENC_OFFSET);

  private SwerveModuleState[] swerveModuleStates;
  public ChassisSpeeds currentMovement;

  // The gyro sensor
  public final double navXPitch() {
    return navX.getPitch();

  }

  public final double navXRoll() {
    return navX.getRoll();
  }

  private static final AHRS navX = new AHRS(SPI.Port.kMXP);
  boolean gyroReset;

  // Odometry class for tracking robot pose with vision
  public SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      SwerveConstants.kDriveKinematics,
      getAngle(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, new Pose2d(0, 0, new Rotation2d(0)));

  /**
   * Creates a new DriveSubsystem.
   */
  public SwerveDrive(Robot m_robot) {
    resetEncoders();
    m_Robot = m_robot;
    rotation.enableContinuousInput(-Math.PI, Math.PI);

    intakeLeft = new AnalogInput(Constants.SwerveConstants.IL_ENC);
    intakeRight = new AnalogInput(Constants.SwerveConstants.IR_ENC);

  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees((navX.getAngle() + 180) * (SwerveConstants.kGyroReversed ? 1.0 : -1.0));
  }

  // FIXME Use correct math, I forgor
  public double getDistanceFromSensor(double voltage) {
    return (1 / (voltage * 1.0 / 1.3) - 0.42);
  }

  public double getIntakeOffsetFromLeft() {
    return ((Constants.ArmConstants.INTAKE_SIZE - (getDistanceFromSensor(intakeLeft.getVoltage())
        + getDistanceFromSensor(intakeRight.getVoltage()))) / 2) + getDistanceFromSensor(intakeLeft.getVoltage());
  }

  public boolean getGyroReset() {
    return gyroReset;
  }

  public void setGyroReset(boolean gyroReset) {
    this.gyroReset = gyroReset;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    m_odometry.update(
        getAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    updateOdometry();

    // System.out.print("xSpeed: " + xAutoSpeed + ";\n ySpeed: " + yAutoSpeed + ";\n
    // rSpeed: " + rAutoSpeed);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void setXWheels() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    navX.reset();
    gyroReset = true;
  }

  // Calculates closest Apriltag for use in autoAlignCube
  public int optimalID() {
    Pose2d robotPose = getPose();
    if (m_Robot.allianceColor == Alliance.Red) {
      if (robotPose.getX() > 8.27) {
        return 5;
      } else {
        return robotPose.getY() <= 4 + 2.098 ? 1 : robotPose.getY() <= 4 + 0.422 ? 2 : 3;
      }
    }

    else {
      if (robotPose.getX() > 8.27) {
        return 4;
      } else {
        return robotPose.getY() <= 4 - 2.098 ? 8 : robotPose.getY() <= 4 - 0.422 ? 7 : 6;
      }
    }

  }

  public void autoAlignCube(double offset, int ID) {

    Pose2d tagPose = VisionConstants.tagPose[ID - 1];

    if (m_Robot.allianceColor == Alliance.Blue) {
      tagPose = new Pose2d(tagPose.getX() + 8.27, tagPose.getY() + 4, tagPose.getRotation());
    } else {
      tagPose = new Pose2d(8.27 - tagPose.getX(), 4 - tagPose.getY(),
          tagPose.getRotation().rotateBy(new Rotation2d(Math.PI)));
    }

    System.out.print("xSpeed " + xAutoSpeed + "; ySpeed " + yAutoSpeed + "; rSpeed " + rAutoSpeed);

    driveTo(
        new Pose2d(tagPose.getX(), tagPose.getY() + offset * tagPose.getRotation().getCos(), tagPose.getRotation()));
  }

  public void driveTo(Pose2d targetPose2d) {
    double xAutoSpeed = transformX.calculate(getPose().getX(), targetPose2d.getX());
    double yAutoSpeed = transformY.calculate(getPose().getY(), targetPose2d.getY());
    double rAutoSpeed = rotation.calculate(getAngle().getRadians(), targetPose2d.getRotation().getRadians());

    // Max Speeds
    xAutoSpeed = MathUtil.clamp(xAutoSpeed, -SwerveConstants.autoAlignMaxSpeedMetersPerSecond,
        SwerveConstants.autoAlignMaxSpeedMetersPerSecond);
    yAutoSpeed = MathUtil.clamp(yAutoSpeed, -SwerveConstants.autoAlignMaxSpeedMetersPerSecond,
        SwerveConstants.autoAlignMaxSpeedMetersPerSecond);

    drive(xAutoSpeed, yAutoSpeed, rAutoSpeed, true);
  }

  public void autoAlignConeOrFeeder(double offset) {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    // double tx =
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    // double thor =
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
    // double xSpeed = transformX.calculate(tx,0);
    // double ySpeed = transformY.calculate(thor,0);
    // double rSpeed = rotation.calculate(getAngle().getRadians(), Math.PI);
    int ID = optimalID();

    double finalOffset = ID == 5 || ID == 4 ? Constants.VisionConstants.FEEDER_OFFSET_LEFT.getY()
        : Constants.VisionConstants.CONE_OFFSET_LEFT;

    autoAlignCube(finalOffset * offset, optimalID());
    // drive(xSpeed, ySpeed, rSpeed, false);
  }

  public void updateOdometry() {

    if (m_Robot.allianceColor == Alliance.Blue) {
      tempRobotPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose.wpiblue")
          .getDoubleArray(new double[1]);
    } else {
      tempRobotPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired")
          .getDoubleArray(new double[1]);
    }

    if (tv > 0.5 && tempRobotPose.length > 1) {
      visionPose = new Pose2d(tempRobotPose[0], tempRobotPose[1], Rotation2d.fromDegrees(tempRobotPose[5]));
      m_odometry.addVisionMeasurement(visionPose,
          Timer.getFPGATimestamp() - tempRobotPose[6] / 1000.0 /*- (tl/1000.0) - (cl/1000.0)*/);
    }
    // System.out.println(robotPose[0] + robotPose[1] + robotPose[5]);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public static double getHeading() {
    double heading = Math.IEEEremainder(navX.getAngle(), 360) * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
    return heading;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navX.getRate() * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void zeroWheels() {
    m_frontLeft.resetWheel();
    m_rearLeft.resetWheel();
    m_frontRight.resetWheel();
    m_rearRight.resetWheel();
  }

  // Assuming this method is part of a drivetrain subsystem that provides the
  // necessary methods
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            PathPlannerState state = PathPlannerTrajectory.transformStateForAlliance(traj.getInitialState(),
                m_Robot.allianceColor);
            this.resetOdometry(new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
          }
        }),
        new PPSwerveControllerCommand(
            traj,
            this::getPose, // Pose supplier
            SwerveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDController(1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use
                                        // feedforwards.
            new PIDController(1, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(1, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will
                                        // only use feedforwards.
            this::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            this // Requires this drive subsystem
        ));

  }

  public void setPresetEnabled(boolean enabled, double desiredHeading) {
    presetEnabled = enabled;
    rotationPreset = desiredHeading;
  }

  public void setPresetEnabled(boolean enabled) {
    presetEnabled = enabled;
  }

  public boolean getPresetEnabled() {
    return presetEnabled;
  }

  public double getRotationPreset() {
    return rotationPreset;
  }

  public void updateSmartDashBoard() {
    SmartDashboard.putNumber("FLSteering", m_frontLeft.m_absoluteEncoder.getAngle());
    SmartDashboard.putNumber("FRSteering", m_frontRight.m_absoluteEncoder.getAngle());
    SmartDashboard.putNumber("BLSteering", m_rearLeft.m_absoluteEncoder.getAngle());
    SmartDashboard.putNumber("BRSteering", m_rearRight.m_absoluteEncoder.getAngle());
    SmartDashboard.putNumber("FLSteeringDeg", m_frontLeft.m_absoluteEncoder.getAngle() * 180.0 / Math.PI);
    SmartDashboard.putNumber("FRSteeringDeg", m_frontRight.m_absoluteEncoder.getAngle() * 180.0 / Math.PI);
    SmartDashboard.putNumber("BLSteeringDeg", m_rearLeft.m_absoluteEncoder.getAngle() * 180.0 / Math.PI);
    SmartDashboard.putNumber("BRSteeringDeg", m_rearRight.m_absoluteEncoder.getAngle() * 180.0 / Math.PI);
    SmartDashboard.putNumber("FLSteerNEO", m_frontLeft.m_turningEncoder.getPosition());
    SmartDashboard.putNumber("FRSteerNEO", m_frontRight.m_turningEncoder.getPosition());
    SmartDashboard.putNumber("BLSteerNEO", m_rearLeft.m_turningEncoder.getPosition());
    SmartDashboard.putNumber("BRSteerNEO", m_rearRight.m_turningEncoder.getPosition());
    SmartDashboard.putNumber("FLneo", m_frontLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("FRneo", m_frontRight.getState().angle.getRadians());
    SmartDashboard.putNumber("BLneo", m_rearLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("BRneo", m_rearRight.getState().angle.getRadians());
    SmartDashboard.putNumber("x", getPose().getTranslation().getX());
    SmartDashboard.putNumber("y", getPose().getTranslation().getY());
    SmartDashboard.putNumber("r", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("GYRO ANGLE", navX.getAngle());
    SmartDashboard.putNumber("TurnRate", getTurnRate());
    SmartDashboard.putNumber("Limelight Pipeline", NetworkTableInstance.getDefault()
        .getTable("limelight").getEntry("getpipe").getDouble(0));
    SmartDashboard.putNumber("Has Target?", tv);
    SmartDashboard.putNumber("xSpeed", xAutoSpeed);
    SmartDashboard.putNumber("ySpeed", yAutoSpeed);
    SmartDashboard.putNumber("rSpeed", rAutoSpeed);
    SmartDashboard.putNumber("pitch", navX.getPitch());
    SmartDashboard.putNumber("roll", navX.getRoll());
    SmartDashboard.putNumber("yaw", navX.getYaw());
    SmartDashboard.putString("Alliance Color", m_Robot.allianceColor.toString());
    SmartDashboard.putNumber("Vision x", tempRobotPose[0]);
    SmartDashboard.putNumber("Vision y", tempRobotPose[1]);
    SmartDashboard.putNumber("Vision r", tempRobotPose[5]);

    SmartDashboard.putNumber("tempRobotPose length", tempRobotPose.length);

  }
}