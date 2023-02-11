// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int LIMELIGHT = 18;

  public static final class SwerveConstants {
    // Port which the steering encoders are plugged into
    public static final int FL_ENCODER = 0;
    public static final int FR_ENCODER = 1;
    public static final int BL_ENCODER = 2;
    public static final int BR_ENCODER = 3;

    // add offsets for absolute encoders
    public static final int FL_OFFSET = 0;
    public static final int FR_OFFSET = 0;
    public static final int BL_OFFSET = 0;
    public static final int BR_OFFSET = 0;

    public static final double kMaxSpeedMetersPerSecond = 4.42;
    public static final double climbMaxSpeedMetersPerSecond = 0.5;
    public static final double autoAlignMaxSpeedMetersPerSecond = 0.2;

    public static final boolean kGyroReversed = true;

    // encoder's aren't reversed
    public static final boolean frontLeftSteerEncoderReversed = false;
    public static final boolean backLeftSteerEncoderReversed = false;
    public static final boolean frontRightSteerEncoderReversed = false;
    public static final boolean backRightSteerEncoderReversed = false;

    // Distance between centers of right and left wheels on robot in meters
    public static final double kTrackWidth = 0.476;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.527;
    // Distance between front and back wheels on robot

    // kinematics constructor with module positions as arguments
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 10 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 10 * Math.PI;

    public static final double kDriveEncoderCPR = (6.75);
    public static final double kSteerEncoderCPR = (150d / 7);

    // adjust for calibration
    // 2/25/21 - 0.12584
    public static final double kWheelDiameterMeters = .1016;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kDriveEncoderCPR;

    public static final double kSteerEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kSteerEncoderCPR;
    // FIXME Change the encoders to correct ports
    public static final int FL_ENCODER = 2;
    public static final int FR_ENCODER = 3;
    public static final int BL_ENCODER = 1;
    public static final int BR_ENCODER = 0;
    // FIXME Change the offsets to current encoder values
    public final static double FL_ENC_OFFSET = 143; // 183 TODO: 180 off??
    public final static double FR_ENC_OFFSET = 141; // 179
    public final static double BL_ENC_OFFSET = 153; // 221 TODO: 180 off??
    public final static double BR_ENC_OFFSET = 103; // 241
  }
  public static final class visionConstants {
    // Only for the Red Alliance Wall
    public final static Translation2d scoringOffset = new Translation2d(-0.762,0);
    public final static Translation2d feederOffsetLeft = new Translation2d(-0.41275,0.5334);
    public final static double coneOffsetLeft = 0.5588;
    // This is for both
    public final static Pose2d[] tagPose = new Pose2d[] {
      new Pose2d(7.24310 + scoringOffset.getX(), -2.93659, new Rotation2d(0)),
      new Pose2d(7.24310 + scoringOffset.getX(), 1.26019, new Rotation2d(0)),
      new Pose2d(7.24310 + scoringOffset.getX(), 0.41621, new Rotation2d(0)),
      new Pose2d(7.90832 + feederOffsetLeft.getX(), 2.74161, new Rotation2d(0)),
      new Pose2d(-7.90832 - feederOffsetLeft.getX(), 2.74161, new Rotation2d(Math.PI)),
      new Pose2d(-7.24310 - scoringOffset.getX(), 0.41621, new Rotation2d(Math.PI)),
      new Pose2d(-7.24310 - scoringOffset.getX(), -1.26019, new Rotation2d(Math.PI)),
      new Pose2d(-7.24310 - scoringOffset.getX(), -2.93659, new Rotation2d(Math.PI))
      
    };


  }
  public static final class armConstants  {

    // FIXME Change the offsets to current encoder values
    public final static double SHOULDER_OFFSET = 0;
    public final static double ELBOW_OFFSET = 0;
    public final static double WRIST_OFFSET = 0;
    // Gear ratios
    public final static double GR_SHOULDER = 75 * (64.0/14);
    public final static double GR_ELBOW = 45 * (80.0/52);
    // WILL GORGEN's SUGGESTED GEAR RATIO
    public final static double GR_WRIST = 200;

    // ARM LENGTHS
    public final static double SHOULDER_LENGTH = 38 * 0.0254;
    public final static double ELBOW_LENGTH = 34 * 0.0254;
  }
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // changing here -- try raising gains further
    public static final double kPXController = 2.8;
    public static final double kPYController = 2.8;
    public static final double kDXController = 0;
    public static final double kDYController = 0;
    public static final double kPThetaController = 3;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  }

  
  public static final class Electrical {
    // Swerve Motor Controller CAN ID's
    // FIXME Change to correct CAN ID's and give the encoders CAN ID's
    public static final int FL_DRIVE = 13;
    public static final int FR_DRIVE = 6;
    public static final int BL_DRIVE = 11;
    public static final int BR_DRIVE = 8;
    public static final int FL_STEER = 12;
    public static final int FR_STEER = 7;
    public static final int BL_STEER = 10;
    public static final int BR_STEER = 9;
    // FIXME Change to correct CAN ID's And give the encoders CAN ID's
    public static final int SHOULDER = 20;
    public static final int ELBOW = 21;
    public static final int WRIST = 22;
    public static final int FINGER = 23;

    public static final int ROLLERS = 99;
  }

  public static class JoystickButtons {
    public static final XboxController m_driverController = new XboxController(0);
    public static final XboxController m_operatorController = new XboxController(1);

    public static final JoystickButton opA = new JoystickButton(m_operatorController, 1);
    public static final JoystickButton opB = new JoystickButton(m_operatorController, 2);
    public static final JoystickButton opX = new JoystickButton(m_operatorController, 3);
    public static final JoystickButton opY = new JoystickButton(m_operatorController, 4);
    public static final JoystickButton oplBump = new JoystickButton(m_operatorController, 5);
    public static final JoystickButton oprBump = new JoystickButton(m_operatorController, 6);
    public static final JoystickButton oplWing = new JoystickButton(m_operatorController, 7);
    public static final JoystickButton oprWing = new JoystickButton(m_operatorController, 8);
    public static final JoystickButton oplJoy = new JoystickButton(m_operatorController, 9);
    public static final JoystickButton oprJoy = new JoystickButton(m_operatorController, 10);
    public static final POVButton opDpadD = new POVButton(m_operatorController, 180);
    public static final POVButton opDpadU = new POVButton(m_operatorController, 0);
    public static final POVButton opDpadL = new POVButton(m_operatorController, 270);
    public static final POVButton opDpadR = new POVButton(m_operatorController, 90);

    public static final JoystickButton dA = new JoystickButton(m_driverController, 1);
    public static final JoystickButton dB = new JoystickButton(m_driverController, 2);
    public static final JoystickButton dX = new JoystickButton(m_driverController, 3);
    public static final JoystickButton dY = new JoystickButton(m_driverController, 4);
    public static final JoystickButton dlBump = new JoystickButton(m_driverController, 5);
    public static final JoystickButton drBump = new JoystickButton(m_driverController, 6);
    public static final JoystickButton dlWing = new JoystickButton(m_driverController, 7);
    public static final JoystickButton drWing = new JoystickButton(m_driverController, 8);
    public static final JoystickButton dlJoy = new JoystickButton(m_driverController, 9);
    public static final JoystickButton drJoy = new JoystickButton(m_driverController, 10);
  }

}
