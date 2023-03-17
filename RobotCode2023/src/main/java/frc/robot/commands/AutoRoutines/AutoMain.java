package frc.robot.commands.AutoRoutines;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.WaitBehavior;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.gameplay.automations.Balance;
import frc.robot.commands.gameplay.automations.ArmPositions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

public class AutoMain extends CommandBase {

        SwerveDrive m_Drive;
        Arm m_Arm;
        Intake m_Intake;
        SwerveAutoBuilder autoBuilder;

        // Commands for AutoRoutines
        // Command scoreHigh;
        // Command scoreMiddle;
        // Command scoreLow;

        // Command balance;

        public AutoMain(SwerveDrive m_Drive, Arm m_Arm, Intake m_Intake) {
                // Class Variables
                this.m_Drive = m_Drive;
                this.m_Arm = m_Arm;
                this.m_Intake = m_Intake;
                
                autoBuilder = new SwerveAutoBuilder(
                        m_Drive::getPose, // Pose2d supplier
                        m_Drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                        SwerveConstants.kDriveKinematics, // SwerveDriveKinematics
                        new PIDConstants(1, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                        new PIDConstants(1, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
                        m_Drive::setModuleStates, // Module states consumer used to output to the drive subsystem
                        AutoConstants.eventMap,
                        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                        m_Drive // The drive subsystem. Used to properly set the requirements of path following commands
                        );
        }

        // Base Commands
        public final Command scoreHigh() {
                return (new ArmPositions(Constants.ArmConstants.HIGH_POSITION, m_Arm)
                                .andThen(runOutakeForTime(0.3))
                                .andThen(new ArmPositions(Constants.ArmConstants.DEFAULT_POSITION, m_Arm)));

        }
        public final Command scoreHighReturnLowCube() {
                return (new ArmPositions(Constants.ArmConstants.HIGH_POSITION, m_Arm)
                                .andThen(runOutakeForTime(0.3))
                                .andThen(new ArmPositions(Constants.ArmConstants.LOW_CUBE_POSITION, m_Arm)));

        }

        public final Command scoreMiddle() {
                return (new ArmPositions(Constants.ArmConstants.MID_POSITION, m_Arm)
                                .andThen(runOutakeForTime(0.3))
                                .andThen(new ArmPositions(Constants.ArmConstants.DEFAULT_POSITION, m_Arm)));
        }

        public final Command scoreLow() {
                return (new ArmPositions(Constants.ArmConstants.LOW_UPRIGHT_CONE_POSITION, m_Arm)
                                .andThen(runOutakeForTime(0.3))
                                .andThen(new ArmPositions(Constants.ArmConstants.DEFAULT_POSITION, m_Arm)));
        }

        public Command runIntakeForTime(double time) {
                return new RunCommand(m_Intake::runIntake, m_Intake).withTimeout(time)
                                .andThen(new InstantCommand(m_Intake::intakeOff, m_Intake));
        }

        public Command runOutakeForTime(double time) {
                return new RunCommand(m_Intake::runOutake, m_Intake).withTimeout(time)
                                .andThen(new InstantCommand(m_Intake::intakeOff, m_Intake));
        }

        public final Command balance() {
                return (new Balance(m_Drive));
        }

        // One Cone Autos
        public Command topOneConeLeaveCommand() {
                PathPlannerTrajectory topOneConeLeave = PathPlanner.loadPath("topOneConeLeave",
                                new PathConstraints(3, 1.5));

                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(topOneConeLeave, true),
                                                topOneConeLeave.getMarkers(),
                                                Constants.AutoConstants.eventMap));
        }

        public Command topOneConeBalanceCommand() {
                PathPlannerTrajectory topOneConeBalance = PathPlanner.loadPath("topOneConeBalance",
                                new PathConstraints(3, 1.5));

                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(topOneConeBalance, true),
                                                topOneConeBalance.getMarkers(),
                                                Constants.AutoConstants.eventMap))
                                .andThen(balance());
        }

        public Command middleOneConeLeaveBottomCommand() {
                PathPlannerTrajectory middleOneConeLeave = PathPlanner.loadPath("middleOneConeLeaveBottom",
                                new PathConstraints(3.5, 2));

                // return scoreHigh()
                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(middleOneConeLeave, true),
                                                middleOneConeLeave.getMarkers(),
                                                Constants.AutoConstants.eventMap))
                                .andThen(balance());
        }

        public Command middleOneConeLeaveTopCommand() {
                PathPlannerTrajectory middleOneConeLeave = PathPlanner.loadPath("middleOneConeLeaveTop",
                                new PathConstraints(3.5, 2));

                // return scoreHigh()
                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(middleOneConeLeave, true),
                                                middleOneConeLeave.getMarkers(),
                                                Constants.AutoConstants.eventMap))
                                .andThen(balance());
        }

        public Command middleOneConeBalanceCommand() {
                PathPlannerTrajectory middleOneConeBalance = PathPlanner.loadPath("middleOneConeBalance",
                                new PathConstraints(1.5, 0.75));

                // return scoreHigh()
                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(middleOneConeBalance, true),
                                                middleOneConeBalance.getMarkers(),
                                                Constants.AutoConstants.eventMap))
                                .andThen(balance());

        }

        public Command bottomOneConeLeaveCommand() {
                PathPlannerTrajectory bottomOneConeLeave = PathPlanner.loadPath("bottomOneConeLeave",
                                new PathConstraints(3, 1.5));

                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(bottomOneConeLeave, true),
                                                bottomOneConeLeave.getMarkers(),
                                                Constants.AutoConstants.eventMap));

        }

        public Command bottomOneConeBalanceCommand() {
                PathPlannerTrajectory bottomOneConeBalance = PathPlanner.loadPath("bottomOneConeBalance",
                                new PathConstraints(3, 2));

                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(bottomOneConeBalance, true),
                                                bottomOneConeBalance.getMarkers(),
                                                Constants.AutoConstants.eventMap)
                                                .andThen(new Balance(m_Drive)));

        }

        // Two Cone Autos

        public Command topOneConeOneCube() {
                PathPlannerTrajectory topTwoConeLeave = PathPlanner.loadPath("topOneConeOneCube",
                                new PathConstraints(4.25, 3.25));

                return scoreHighReturnLowCube()
                                .andThen((new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(topTwoConeLeave, true),
                                                topTwoConeLeave.getMarkers(),
                                                Constants.AutoConstants.eventMap))
                                                .alongWith(new ArmPositions(Constants.ArmConstants.LOW_CUBE_POSITION, m_Arm))
                                                .alongWith(runIntakeForTime(4.3)))
                                .andThen(scoreHigh());

        }
        public Command topOneConeOneCubeBalance() {
                PathPlannerTrajectory topTwoConeLeave = PathPlanner.loadPath("topOneConeOneCubeBalance",
                                new PathConstraints(Constants.SwerveConstants.kMaxSpeedMetersPerSecond, 3.5));

                return scoreHighReturnLowCube()
                                .andThen((new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(topTwoConeLeave, true),
                                                topTwoConeLeave.getMarkers(),
                                                Constants.AutoConstants.eventMap))
                                                .alongWith(new ArmPositions(Constants.ArmConstants.LOW_CUBE_POSITION, m_Arm))
                                                .alongWith(runIntakeForTime(4.3)))
                                .andThen(new Balance(m_dri));

        }

        // public Command topOneConeOneCubeCommandWithMarkers()    {
        //         List<PathPlannerTrajectory> pathGroup = 
        //                 PathPlanner.loadPathGroup("topOneConeOneCube", new PathConstraints(4, 3));
                
        //         return new ArmPositions(Constants.ArmConstants.HIGH_POSITION, m_Arm)
        //         .andThen(runOutakeForTime(0.3))
        //         .andThen(new ArmPositions(Constants.ArmConstants.LOW_CUBE_POSITION, m_Arm))
        //                         .alongWith(new WaitCommand(1)
        //                         .andThen(autoBuilder.fullAuto(pathGroup)))
        //                         .andThen(scoreHigh());
                

        // }       

}