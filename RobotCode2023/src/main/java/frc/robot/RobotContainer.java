// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.JoystickButtons;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveDrive m_robotDrive = new SwerveDrive();

  // The driver's controller
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    JoystickButtons.m_driverController.getLeftY(),
                    JoystickButtons.m_driverController.getLeftX(),
                    JoystickButtons.m_driverController.getRightX(),
                    false),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {}

<<<<<<< Updated upstream
=======
    // Driver Controls
    // 
    JoystickButtons.dA.whileTrue(new RunCommand( ()-> m_robotDrive.autoAlignCube(0, m_robotDrive.optimalID()),m_robotDrive));
    JoystickButtons.dX.whileTrue(new RunCommand( () -> m_robotDrive.autoAlignConeOrFeeder(-1),m_robotDrive));
    JoystickButtons.dB.whileTrue(new RunCommand( () -> m_robotDrive.autoAlignConeOrFeeder(1), m_robotDrive));
    JoystickButtons.dlWing.onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));
    JoystickButtons.dY.whileTrue(new Balance(m_robotDrive));

    // Operator Controls
    // TODO 5 buttons total plus manual override for operator
    JoystickButtons.oprBump.whileTrue(new RunCommand(m_Intake::runIntake,m_Intake));
    JoystickButtons.oplBump.whileTrue(new RunCommand(m_Intake::runOutake, m_Intake));
    
    JoystickButtons.opY.whileTrue(new armPositions(Constants.armConstants.DEFAULT_POSITION, m_Arm)
      .andThen(new armPositions(Constants.armConstants.INTERMEDIATE_LOW_POSITION, m_Arm)));
    JoystickButtons.opA.whileTrue(new armPositions(Constants.armConstants.INTERMEDIATE_MID_POSITION, m_Arm)
      .andThen(new armPositions(Constants.armConstants.HIGH_POSITION, m_Arm)));
    JoystickButtons.opX.whileTrue(new armPositions(Constants.armConstants.INTERMEDIATE_MID_POSITION, m_Arm)
      .andThen(new armPositions(Constants.armConstants.MID_POSITION, m_Arm)));
    JoystickButtons.opB.whileTrue(new armPositions(Constants.armConstants.INTERMEDIATE_LOW_POSITION, m_Arm)
      .andThen(new armPositions(Constants.armConstants.LOW_POSITION, m_Arm)));
      
  }
>>>>>>> Stashed changes
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(SwerveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            SwerveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
