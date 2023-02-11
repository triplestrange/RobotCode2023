/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Electrical;
import frc.robot.Constants.ModuleConstants;
// import frc.robot.Constants.ModuleConstants;
// import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.visionConstants;

@SuppressWarnings("PMD.ExcessiveImports")
public class SwerveDrive extends SubsystemBase {
  public PIDController transformX = new PIDController(0.05, 0, 0);
  public PIDController transformY = new PIDController(0.05, 0, 0);
  public PIDController rotation = new PIDController(0.05,0,0);
  public double xAutoSpeed = 0;
  public double yAutoSpeed = 0;
  public double rAutoSpeed = 0;

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
  private SwerveModuleState[] initStates;
  private double horPos;
  public ChassisSpeeds currentMovement;

  // The gyro sensor
  public final double navXPitch()  {
  return navX.getPitch();

  }
  private static final AHRS navX = new AHRS(SPI.Port.kMXP);
  boolean gyroReset;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          SwerveConstants.kDriveKinematics,
          getAngle(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  /**
   * Creates a new DriveSubsystem.
   */
  public SwerveDrive() {
    resetEncoders();
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

  public boolean getGyroReset() {
    return gyroReset;
  }

  public void setGyroReset(boolean gyroReset) {
    this.gyroReset = gyroReset;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        getAngle(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
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

    // SmartDashboard.putNumber("xSpeed", xAutoSpeed);
    // SmartDashboard.putNumber("ySpeed", yAutoSpeed);
    // SmartDashboard.putNumber("rSpeed", rAutoSpeed);
    // System.out.print("xSpeed: " + xAutoSpeed + ";\n ySpeed: " + yAutoSpeed + ";\n rSpeed: " + rAutoSpeed);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
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
    if (DriverStation.getAlliance() == Alliance.Red)  {
      if (robotPose.getX() < 0) {
        return 5;
      }
      else  {
        return robotPose.getY() <= -2.098 ? 1 : robotPose.getY() <= -0.422 ? 2 : 3;
      }
    }
    else  {
      if (robotPose.getX() > 0) {
        return 4;
      }
      else  {
        return robotPose.getY() <= -2.098 ? 8 : robotPose.getY() <= -0.422 ? 7 : 6;
      }
      }
      
  }
  // FIXME ADD MAX SDEED LIMITS BEFORE TESTING
  public void autoAlignCube(double offset, int ID) {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    // double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    // double thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
    // TODO May have to change tagPose 
    
    Pose2d tagPose = visionConstants.tagPose[ID - 1];
    xAutoSpeed = transformX.calculate(getPose().getX(),tagPose.getX());
    yAutoSpeed = transformY.calculate(getPose().getY() + offset * tagPose.getRotation().getCos(), tagPose.getY());
    rAutoSpeed = rotation.calculate(getAngle().getRadians(), tagPose.getRotation().getRadians());
    // MAX SPEEDS
    //  if (xAutoSpeed > SwerveConstants.autoAlignMaxSpeedMetersPerSecond) {
    //   xAutoSpeed = SwerveConstants.autoAlignMaxSpeedMetersPerSecond;}
    // else if (xAutoSpeed < -SwerveConstants.autoAlignMaxSpeedMetersPerSecond) {
    // xAutoSpeed = -SwerveConstants.autoAlignMaxSpeedMetersPerSecond;}

    //  if (yAutoSpeed > SwerveConstants.autoAlignMaxSpeedMetersPerSecond) {
    //   yAutoSpeed = SwerveConstants.autoAlignMaxSpeedMetersPerSecond;}
    // else if (yAutoSpeed < -SwerveConstants.autoAlignMaxSpeedMetersPerSecond) {
    // yAutoSpeed = -SwerveConstants.autoAlignMaxSpeedMetersPerSecond;}
    
    // System.out.print("xSpeed " + xSpeed + "; ySpeed " + ySpeed + "; rSpeed " + rSpeed);
    
    drive(xAutoSpeed, yAutoSpeed, rAutoSpeed, true);
    }

    

 public void autoAlignConeOrFeeder(double offset) {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    // double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    // double thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
    // double xSpeed = transformX.calculate(tx,0);
    // double ySpeed = transformY.calculate(thor,0);
    // double rSpeed = rotation.calculate(getAngle().getRadians(), Math.PI);
    int ID = optimalID();

    double finalOffset = ID == 5 || ID == 4 ? Constants.visionConstants.feederOffsetLeft.getY() : Constants.visionConstants.coneOffsetLeft;


    autoAlignCube(finalOffset * offset, optimalID());
    // drive(xSpeed, ySpeed, rSpeed, false);
  }


  public void updateOdometry()  {
  double[] robotPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
  int tv = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getInteger(0);
  if (tv == 1 && robotPose.length == 6)  {resetOdometry(new Pose2d(robotPose[0], robotPose[1], getAngle()));}
  }
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public static double getHeading() {
    double heading = Math.IEEEremainder(navX.getAngle(), 360) * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
    SmartDashboard.putNumber("heading", heading);
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
}