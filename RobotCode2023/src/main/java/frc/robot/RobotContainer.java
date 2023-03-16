// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickButtons;
import frc.robot.commands.AutoRoutines.AutoMain;
import frc.robot.commands.gameplay.automations.Balance;
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
                // One Cone Top
                choose.addOption("Top One Cone Leave",
                                m_Autos.topOneConeLeaveCommand());
                choose.addOption("Top One Cone Balance",
                                m_Autos.topOneConeBalanceCommand());
                // One Cone Mid
                choose.addOption("Middle One Cone Leave Bottom",
                                m_Autos.middleOneConeLeaveBottomCommand());
                choose.addOption("Middle One Cone Leave Top",
                                m_Autos.middleOneConeLeaveTopCommand());
                choose.addOption("Middle One Cone Balance",
                                m_Autos.middleOneConeBalanceCommand());
                // One Cone Bottom
                choose.addOption("Bottom One Cone Leave",
                                m_Autos.bottomOneConeLeaveCommand());
                choose.addOption("Bottom One Cone Balance",
                                m_Autos.bottomOneConeBalanceCommand());
                // Two Game Piece Top
                choose.addOption("Top One Cone One Cube",
                                m_Autos.topOneConeOneCube());

                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new DriveTurbo(m_robotDrive));
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
                                                0.1 * JoystickButtons.m_operatorController.getLeftY(),
                                                -0.1 * JoystickButtons.m_operatorController.getRightY(),
                                                0.1 * (JoystickButtons.m_operatorController.getRightTriggerAxis()
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
                //
                JoystickButtons.dA
                                .whileTrue(new RunCommand(() -> m_robotDrive.autoAlignCube(0, m_robotDrive.optimalID()),
                                                m_robotDrive));
                JoystickButtons.dX
                                .whileTrue(new RunCommand(() -> m_robotDrive.autoAlignConeOrFeeder(-1), m_robotDrive));
                JoystickButtons.dB.whileTrue(new RunCommand(() -> m_robotDrive.autoAlignConeOrFeeder(1), m_robotDrive));
                JoystickButtons.dlBump.whileTrue(new DriveNormal(m_robotDrive));
                JoystickButtons.drBump.whileTrue(new DriveSlow(m_robotDrive));
                JoystickButtons.dlWing.onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));
                JoystickButtons.dY.whileTrue(new Balance(m_robotDrive));
                JoystickButtons.drWing.onTrue(new InstantCommand(m_robotDrive::setXWheels, m_robotDrive));

                // Operator Controls
                JoystickButtons.oprBump.whileTrue(new RunCommand(m_Intake::runIntake, m_Intake));
                JoystickButtons.oplBump.whileTrue(new RunCommand(m_Intake::runOutake, m_Intake));
                // JoystickButtons.dDpadL.toggleOnTrue(new DriveDir(m_robotDrive, 0));
                // JoystickButtons.dDpadD.toggleOnTrue(new DriveDir(m_robotDrive, 90));

                JoystickButtons.dDpadL.whileTrue(new DriveDir(m_robotDrive, 90));
                JoystickButtons.dDpadD.whileTrue(new DriveDir(m_robotDrive, 0));


                // ArmTrajectory(Constants.ArmConstants.LOW_UPRIGHT_CONE_POSITION, m_Arm));
                // Y | high: 29.25, 29.11, 34
                // X | mid: -
                // dR| cube: -42.7,-133.7,43.5
                // dU| cone upright: -34.69, -139.75, 37.55
                // B | feeder slope: -2, -162.5, 115.2
                // dD| cone lying: -56.45, -139.75, 95.84
                // A | default: -0.6, -169.63, 137.6
                // dL| feed slide

                JoystickButtons.opY.whileTrue(// high
                                new ArmPositions(
                                                Constants.ArmConstants.HIGH_POSITION,
                                                m_Arm));
                JoystickButtons.opX.whileTrue(// mid
                                new ArmPositions(
                                                Constants.ArmConstants.MID_POSITION,
                                                m_Arm));
                JoystickButtons.opDpadR.whileTrue(// cube
                                new ArmPositions(
                                                new JointAngles(Math.toRadians(-42.7), Math.toRadians(-133.7),
                                                                Math.toRadians(37.55)),
                                                m_Arm));
                JoystickButtons.opDpadU.whileTrue(
                                // coneupright
                                new ArmPositions(
                                                Constants.ArmConstants.LOW_UPRIGHT_CONE_POSITION,
                                                m_Arm));
                JoystickButtons.opB.whileTrue(// feeder slope
                                new ArmPositions(
                                                new JointAngles(Math.toRadians(-2), Math.toRadians(-162.5),
                                                                Math.toRadians(115.2)),
                                                m_Arm));
                JoystickButtons.opDpadD.whileTrue(// lyingcone
                                new ArmPositions(
                                                Constants.ArmConstants.LOW_LYING_CONE_POSITION,
                                                m_Arm));
                JoystickButtons.opA.whileTrue(// default
                                new ArmPositions(
                                                Constants.ArmConstants.DEFAULT_POSITION,
                                                m_Arm));
                JoystickButtons.opDpadL.whileTrue(// feeder slider
                                new ArmPositions(
                                                new JointAngles(Math.toRadians(-21.68), Math.toRadians(-33.43),
                                                                Math.toRadians(-88.5)),
                                                m_Arm));
        }
        /**
         * 
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

}
