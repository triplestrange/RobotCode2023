// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.JoystickButtons;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AutoRoutines.BottomOneConeLeave;
import frc.robot.commands.AutoRoutines.MiddleOneConeBalance;
import frc.robot.commands.AutoRoutines.TopOneConeLeave;
import frc.robot.commands.AutoRoutines.TopOneConeLeave;
import frc.robot.commands.gameplay.automations.Balance;
import frc.robot.commands.gameplay.automations.Balance2;
import frc.robot.commands.gameplay.automations.DriveNormal;
import frc.robot.commands.gameplay.automations.DriveTurbo;
import frc.robot.commands.gameplay.automations.FilteredDrive;
import frc.robot.commands.gameplay.automations.armPositions;
import frc.robot.commands.gameplay.automations.armTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Arm.JointAngles;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Robot m_Robot;
  public final SwerveDrive m_robotDrive;
  public final Arm m_Arm = new Arm();
  private final Intake m_Intake = new Intake();
  private final SendableChooser<Command> choose;
  // The driver's controller
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot m_robot, SendableChooser<Command> choose) {
    m_Robot = m_robot;
    m_robotDrive = new SwerveDrive(m_robot);

    this.choose = choose;

    choose.addOption("Top One Cone Leave", new TopOneConeLeave(m_robotDrive, m_Arm, m_Intake));
    choose.addOption("Bottom One Cone Leave", new BottomOneConeLeave(m_robotDrive, m_Arm, m_Intake));
    choose.addOption("Middle One Cone Balance", new MiddleOneConeBalance(m_robotDrive, m_Arm, m_Intake));
    // Configure the button bindings
    configureButtonBindings();
  
    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      // TODO fine tune motor speeds
      new DriveNormal(m_robotDrive)
    );
    // m_robotDrive.setDefaultCommand(new FilteredDrive(m_robotDrive, 
            // XBOX
            // () -> JoystickButtons.m_driverController.getLeftY() * 5,
            // () -> JoystickButtons.m_driverController.getLeftX() * 5,
            // () -> JoystickButtons.m_driverController.getRightX() * 5));
            // Logitech
            // () -> JoystickButtons.m_driverController.getLeftY() * 2, 
            // () -> JoystickButtons.m_driverController.getLeftX() * 2, 
            // () -> JoystickButtons.m_driverController.getRawAxis(2) * 5));
        


    m_Arm.setDefaultCommand(new RunCommand(
    //     () -> m_Arm.moveArm(
    //         0.1 * JoystickButtons.m_operatorController.getLeftY(),
    //         -0.1 * JoystickButtons.m_operatorController.getRightY(), 
    //         0.1 * (JoystickButtons.m_operatorController.getLeftTriggerAxis() - JoystickButtons.m_operatorController.getRightTriggerAxis())),
    //     m_Arm
    // ));

            () -> m_Arm.moveArm(
            0.1 * JoystickButtons.m_operatorController.getLeftY(),
            -0.1 * JoystickButtons.m_operatorController.getRightY(), 
            0.1 * (JoystickButtons.m_operatorController.getRightTriggerAxis() - JoystickButtons.m_operatorController.getLeftTriggerAxis())),
        m_Arm
    ));
    m_Intake.setDefaultCommand(new RunCommand(m_Intake::intakeOff, m_Intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver Controls
    // 
    JoystickButtons.dA.whileTrue(new RunCommand( ()-> m_robotDrive.autoAlignCube(0, m_robotDrive.optimalID()),m_robotDrive));
    JoystickButtons.dX.whileTrue(new RunCommand( () -> m_robotDrive.autoAlignConeOrFeeder(-1),m_robotDrive));
    JoystickButtons.dB.whileTrue(new RunCommand( () -> m_robotDrive.autoAlignConeOrFeeder(1), m_robotDrive));
    JoystickButtons.dlBump.whileTrue(new DriveTurbo(m_robotDrive));
    JoystickButtons.dlWing.onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));
    JoystickButtons.dY.whileTrue(new Balance(m_robotDrive));
    JoystickButtons.drWing.onTrue(new InstantCommand(m_robotDrive::setXWheels, m_robotDrive));

    // Operator Controls
    // TODO 5 buttons total plus manual override for operator
    JoystickButtons.oprBump.whileTrue(new RunCommand(m_Intake::runIntake,m_Intake));
    JoystickButtons.oplBump.whileTrue(new RunCommand(m_Intake::runOutake, m_Intake));
    
    JoystickButtons.opB.whileTrue(new armTrajectory(Constants.armConstants.LOW_UPRIGHT_CONE_POSITION, m_Arm));
    JoystickButtons.opY.whileTrue(new armTrajectory(Constants.armConstants.HIGH_POSITION, m_Arm));
    JoystickButtons.opX.whileTrue(new armTrajectory(Constants.armConstants.MID_POSITION, m_Arm));
    JoystickButtons.opA.whileTrue(new armTrajectory(Constants.armConstants.LOW_LYING_CONE_POSITION, m_Arm));
    JoystickButtons.opDpadD.whileTrue(new armTrajectory(Constants.armConstants.DEFAULT_POSITION, m_Arm));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   
}
  