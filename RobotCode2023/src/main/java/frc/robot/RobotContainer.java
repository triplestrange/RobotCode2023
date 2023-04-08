// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickButtons;
// import frc.robot.commands.AutoRoutines.AutoMain;
import frc.robot.commands.gameplay.automations.Balance;
import frc.robot.commands.gameplay.automations.DriveTo;
import frc.robot.commands.AutoRoutines.AutoMain;
import frc.robot.commands.drive.*;
import frc.robot.commands.gameplay.automations.ArmPositions;
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
        public final AutoMain m_Autos;

        // The driver's controller
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer(Robot m_Robot, SendableChooser<Command> choose) {
                this.m_Robot = m_Robot;
                m_robotDrive = new SwerveDrive(m_Robot);
                this.m_Autos = new AutoMain(m_robotDrive, m_Arm, m_Intake);

                this.choose = choose;
                // Working Consistently
                // One Cone Top
                choose.addOption("Feeder One Cone Leave",
                                m_Autos.topOneConeLeaveCommand());
                choose.addOption("Feeder One Cone Balance",
                                m_Autos.topOneConeBalanceCommand());
                // One Cone Mid
                choose.addOption("Middle One Cone Leave Bottom",
                                m_Autos.middleOneConeLeaveBottomCommand());
                choose.addOption("Middle One Cone Leave Top",
                                m_Autos.middleOneConeLeaveTopCommand());
                choose.addOption("Middle One Cone Balance",
                                m_Autos.middleOneConeBalanceCommand());
                // One Cone Bottom
                choose.addOption("!Feeder One Cone Leave",
                                m_Autos.bottomOneConeLeaveCommand());
                choose.addOption("!Feeder One Cone Balance",
                                m_Autos.bottomOneConeBalanceCommand());

                // Not Working Consistently
                // Two Game Piece Top
                choose.addOption("Feeder One Cone One Cube",
                                m_Autos.topOneConeOneCube());
                choose.addOption("Feeder One Cone One Cube Balance",
                                m_Autos.topOneConeOneCubeBalance());
                // Two Game Piece Bottom
                choose.addOption("!Feeder One Cone One Cube",
                                m_Autos.bottomOneConeOneCube());
                choose.addOption("!Feeder One Cone One Cube Balance",
                                m_Autos.bottomOneConeOneCubeBalance());

                // Three Game Piece Top
                choose.addOption("Feeder Low Three Cube",
                                m_Autos.topLowThreeCube());
                // Three Game Piece Bottom
                choose.addOption("!Feeder Low Three Cube",
                                m_Autos.bottomLowThreeCube());
                choose.addOption("score high", m_Autos.scoreHigh());
                // Testing
                choose.addOption("Simultaneous Movement Test",
                                m_Autos.testSimultaneousMovement());
                choose.setDefaultOption("score High", m_Autos.scoreHigh());
                // // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new DefaultDrive(m_robotDrive, 2.5, 1));
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
                                // () -> m_Arm.moveArm(
                                // 0.1 * JoystickButtons.m_operatorController.getLeftY(),
                                // -0.1 * JoystickButtons.m_operatorController.getRightY(),
                                // 0.1 * (JoystickButtons.m_operatorController.getLeftTriggerAxis() -
                                // JoystickButtons.m_operatorController.getRightTriggerAxis())),
                                // m_Arm
                                // ));

                                () -> m_Arm.moveArm(
                                                0.6 * JoystickButtons.m_operatorController.getLeftY(),
                                                -0.8 * JoystickButtons.m_operatorController.getRightY(),
                                                0.2 * (JoystickButtons.m_operatorController.getRightTriggerAxis()
                                                                - JoystickButtons.m_operatorController
                                                                                .getLeftTriggerAxis())),
                                m_Arm));
                m_Intake.setDefaultCommand(new RunCommand(m_Intake::intakeOff, m_Intake));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {

                // Driver Controls
                // Auto Align
                JoystickButtons.dA
                                .whileTrue(new DriveTo(0, m_robotDrive, m_Robot));
                JoystickButtons.dX
                                .whileTrue(new DriveTo(1, m_robotDrive, m_Robot));
                JoystickButtons.dB.whileTrue(new DriveTo(-1, m_robotDrive, m_Robot));
                // Drive Commands
                JoystickButtons.drBump.whileTrue(
                                new DefaultDrive(m_robotDrive, 0.75, 1));
                JoystickButtons.dlBump.whileTrue(
                                new DefaultDrive(m_robotDrive, Constants.SwerveConstants.kMaxSpeedMetersPerSecond, 2));

                JoystickButtons.dlWing.onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));
                JoystickButtons.dY.whileTrue(new Balance(m_robotDrive));
                JoystickButtons.drWing.onTrue(new InstantCommand(m_robotDrive::setXWheels, m_robotDrive));

                // new Trigger(() ->
                // Math.abs(JoystickButtons.m_driverController.getLeftTriggerAxis()) > 0.05)
                // .onTrue(new InstantCommand(() -> {
                // m_robotDrive.setPresetEnabled(true, -180.0);
                // }));

                // new Trigger(() ->
                // Math.abs(JoystickButtons.m_driverController.getRightTriggerAxis()) > 0.05)
                // .onTrue(new InstantCommand(() -> {
                // m_robotDrive.setPresetEnabled(true, 0);

                // }));

                // Y | high: 29.25, 29.11, 34
                // X | mid: -
                // dR| cube: -42.7,-133.7,43.5
                // dU| cone upright: -34.69, -139.75, 37.55
                // B | feeder slope: -2, -162.5, 115.2
                // dD| cone lying: -56.45, -139.75, 95.84
                // A | default: -0.6, -169.63, 137.6
                // dL| feed slide -2.78, -73.08, -57.43

                // Operator Controls
                JoystickButtons.oprBump.whileTrue(new RunCommand(m_Intake::runIntake, m_Intake));
                JoystickButtons.oplBump.whileTrue(new RunCommand(m_Intake::runOutake, m_Intake));

                JoystickButtons.opY.whileTrue(// high
                                new ArmPositions(
                                                Constants.ArmConstants.HIGH_POSITION,
                                                m_Arm));
                JoystickButtons.opX.whileTrue(// mid
                                new ArmPositions(
                                                Constants.ArmConstants.MID_POSITION,
                                                m_Arm));
                JoystickButtons.opA.whileTrue(// default
                                new ArmPositions(
                                                Constants.ArmConstants.DEFAULT_POSITION,
                                                m_Arm));
                JoystickButtons.opB.whileTrue(// feeder slope
                                new ArmPositions(
                                                Constants.ArmConstants.SINGLE_FEEDER_STATION,
                                                m_Arm));
                JoystickButtons.opDpadD.whileTrue(// lying cone
                                new ArmPositions(
                                                Constants.ArmConstants.LOW_LYING_CONE_POSITION,
                                                m_Arm));
                JoystickButtons.opDpadR.whileTrue(// cube
                                new ArmPositions(
                                                Constants.ArmConstants.LOW_CUBE_POSITION,
                                                m_Arm));
                JoystickButtons.opDpadU.whileTrue(
                                // coneupright
                                new ArmPositions(
                                                Constants.ArmConstants.LOW_UPRIGHT_CONE_POSITION,
                                                m_Arm));

                JoystickButtons.opDpadL.whileTrue(// feeder slider
                                new ArmPositions(
                                                Constants.ArmConstants.DOUBLE_FEEDER_STATION,
                                                m_Arm));

                // // JoystickButtons.oplWing.whileTrue(new InstantCommand(() -> {
                // // m_robotDrive.setPresetEnabled(true, 0);

                // }));
        }

        /**
         * 
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

}
