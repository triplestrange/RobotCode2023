package frc.robot.commands.AutoRoutines;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.gameplay.automations.Balance;
import frc.robot.commands.gameplay.automations.ArmPositions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Arm.JointAngles;

public class AutoMain extends CommandBase {

        SwerveDrive m_Drive;
        Arm m_Arm;
        Intake m_Intake;
        SwerveAutoBuilder autoBuilder;

        // Commands for AutoRoutines
        public static HashMap<String, Command> eventMap;

        public void eventMapEvents(SwerveDrive m_Drive, Arm m_Arm, Intake m_Intake) {
                // scoring
                eventMap.put("high", new ArmPositions(Constants.ArmConstants.HIGH_POSITION, m_Arm)
                                .andThen(new WaitCommand(1.05)));
                eventMap.put("feeder",
                                new ArmPositions(new JointAngles(
                                                Constants.ArmConstants.DOUBLE_FEEDER_STATION.getShoulderAngle(),
                                                Constants.ArmConstants.DOUBLE_FEEDER_MIDPOINT.getElbowAngle(), 0),
                                                m_Arm));
                eventMap.put("mid", new ArmPositions(Constants.ArmConstants.MID_POSITION, m_Arm));
                eventMap.put("low", new ArmPositions(Constants.ArmConstants.LOW_UPRIGHT_CONE_POSITION, m_Arm));
                eventMap.put("default", new ArmPositions(Constants.ArmConstants.DEFAULT_POSITION, m_Arm));
                eventMap.putIfAbsent("scoreLow",
                                new ArmPositions(new JointAngles(
                                                Constants.ArmConstants.SINGLE_FEEDER_STATION.getShoulderAngle(),
                                                Math.toRadians(-142),
                                                Constants.ArmConstants.LOW_CUBE_POSITION.getWristAngle()), m_Arm));
                // pickup
                eventMap.put("pickupConeUp", new ArmPositions(Constants.ArmConstants.LOW_UPRIGHT_CONE_POSITION, m_Arm));
                eventMap.put("pickupConeDown", new ArmPositions(Constants.ArmConstants.LOW_LYING_CONE_POSITION, m_Arm));
                eventMap.put("pickupCube", new ArmPositions(Constants.ArmConstants.LOW_CUBE_POSITION, m_Arm));
                // intake
                eventMap.put("intake", new InstantCommand(m_Intake::runIntake, m_Intake));
                eventMap.put("outtakeTimed", new RunCommand(m_Intake::runOutake, m_Intake).withTimeout(0.3));
                eventMap.put("outtake", new InstantCommand(m_Intake::runOutake, m_Intake));
                eventMap.put("intakeOff", new InstantCommand(m_Intake::intakeOff));

        };

        public AutoMain(SwerveDrive m_Drive, Arm m_Arm, Intake m_Intake) {
                // Class Variables
                this.m_Drive = m_Drive;
                this.m_Arm = m_Arm;
                this.m_Intake = m_Intake;
                eventMap = new HashMap<>();
                eventMapEvents(m_Drive, m_Arm, m_Intake);
                autoBuilder = new SwerveAutoBuilder(
                                m_Drive::getPose, // Pose2d supplier
                                m_Drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of
                                                        // auto
                                SwerveConstants.kDriveKinematics, // SwerveDriveKinematics
                                new PIDConstants(2, 0, 0.05), // PID constants to correct for translation error (used
                                                              // to create the X and Y PID controllers)
                                new PIDConstants(2, 0.0, 0.0), // PID constants to correct for rotation error (used to
                                                               // create the rotation controller)
                                m_Drive::setModuleStates, // Module states consumer used to output to the drive
                                                          // subsystem
                                eventMap,
                                true, // Should the path be automatically mirrored depending on alliance color.
                                      // Optional, defaults to true
                                m_Drive // The drive subsystem. Used to properly set the requirements of path following
                                        // commands
                );
        }

        // Base Commands
        public final Command scoreHigh() {
                return (new ArmPositions(Constants.ArmConstants.HIGH_POSITION, m_Arm)
                                .andThen(new WaitCommand(1.05))
                                .andThen(runOutakeForTime(0.3))
                                .andThen(new ArmPositions(Constants.ArmConstants.DEFAULT_POSITION, m_Arm)));

        }

        public final Command pidTuner() {
                return (new ArmPositions(new JointAngles(0, Math.toRadians(90), 0), m_Arm)
                                .andThen(new WaitCommand(4))
                                .andThen(new ArmPositions(new JointAngles(0, Math.toRadians(-90), 0), m_Arm))
                                .andThen(new WaitCommand(4)));
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
                // m_Drive.autoPath.getObject("traj").setTrajectory(topOneConeLeave);

                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(topOneConeLeave, true),
                                                topOneConeLeave.getMarkers(),
                                                eventMap));
        }

        public Command topOneConeBalanceCommand() {
                PathPlannerTrajectory topOneConeBalance = PathPlanner.loadPath("topOneConeBalance",
                                new PathConstraints(3, 1.5));
                // m_Drive.autoPath.getObject("traj").setTrajectory(topOneConeBalance);

                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(topOneConeBalance, true),
                                                topOneConeBalance.getMarkers(),
                                                eventMap))
                                .andThen(balance());
        }

        public Command middleOneConeLeaveBottomCommand() {
                PathPlannerTrajectory middleOneConeLeave = PathPlanner.loadPath("middleOneConeLeaveBottom",
                                new PathConstraints(3.5, 2));
                // m_Drive.autoPath.getObject("traj").setTrajectory(middleOneConeLeave);

                // return scoreHigh()
                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(middleOneConeLeave, true),
                                                middleOneConeLeave.getMarkers(),
                                                eventMap))
                                .andThen(balance());
        }

        public Command middleOneConeLeaveTopCommand() {
                PathPlannerTrajectory middleOneConeLeave = PathPlanner.loadPath("middleOneConeLeaveTop",
                                new PathConstraints(3.5, 2));
                // m_Drive.autoPath.getObject("traj").setTrajectory(middleOneConeLeave);

                // return scoreHigh()
                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(middleOneConeLeave, true),
                                                middleOneConeLeave.getMarkers(),
                                                eventMap))
                                .andThen(balance());
        }

        // public Command middleOneConeBalanceCommand() {
        // PathPlannerTrajectory middleOneConeBalance =
        // PathPlanner.loadPath("middleOneConeBalance",
        // new PathConstraints(2, 1.5));
        // // m_Drive.autoPath.getObject("traj").setTrajectory(middleOneConeBalance);

        // // return scoreHigh()
        // return scoreHigh()
        // .andThen(new FollowPathWithEvents(
        // m_Drive.followTrajectoryCommand(middleOneConeBalance, true),
        // middleOneConeBalance.getMarkers(),
        // eventMap))
        // .andThen(balance());

        // }

        public Command middleOneHalfConeBalanceCommand() {
                List<PathPlannerTrajectory> middleOneHalfConeBalance = PathPlanner.loadPathGroup(
                                "middleOneHalfConeBalance",
                                new PathConstraints(2, 1.5));
                // m_Drive.autoPath.getObject("traj").setTrajectory(middleOneConeBalance);

                // return scoreHigh()
                return autoBuilder.fullAuto(middleOneHalfConeBalance)
                                .andThen(balance());

        }

        public Command middleOneConeBalanceCommand() {
                List<PathPlannerTrajectory> middleOneConeBalance = PathPlanner.loadPathGroup("middleOneConeBalance",
                                new PathConstraints(2, 1.5));
                // m_Drive.autoPath.getObject("traj").setTrajectory(middleOneConeBalance);

                // return scoreHigh()
                return autoBuilder.fullAuto(middleOneConeBalance)
                                .andThen(balance());

        }

        public Command bottomOneConeLeaveCommand() {
                PathPlannerTrajectory bottomOneConeLeave = PathPlanner.loadPath("bottomOneConeLeave",
                                new PathConstraints(3, 1.5));
                // m_Drive.autoPath.getObject("traj").setTrajectory(bottomOneConeLeave);

                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(bottomOneConeLeave, true),
                                                bottomOneConeLeave.getMarkers(),
                                                eventMap));

        }

        public Command bottomOneConeBalanceCommand() {
                PathPlannerTrajectory bottomOneConeBalance = PathPlanner.loadPath("bottomOneConeBalance",
                                new PathConstraints(3, 2));
                // m_Drive.autoPath.getObject("traj").setTrajectory(bottomOneConeBalance);

                return scoreHigh()
                                .andThen(new FollowPathWithEvents(
                                                m_Drive.followTrajectoryCommand(bottomOneConeBalance, true),
                                                bottomOneConeBalance.getMarkers(),
                                                eventMap)
                                                .andThen(new Balance(m_Drive)));

        }

        // Two Gamepiece Autos

        public Command topOneConeOneCube() {
                List<PathPlannerTrajectory> topOneConeOneCubeLeave = PathPlanner.loadPathGroup(
                                "topOneConeOneCube",
                                new PathConstraints(3.5,
                                                2.75));
                // for (int i = 0; i < topOneConeOneCubeLeave.size(); i++) {
                // m_Drive.autoPath.getObject("traj" +
                // i).setTrajectory(topOneConeOneCubeLeave.get(i - 1));
                // }
                return autoBuilder.fullAuto(topOneConeOneCubeLeave);

        }

        public Command bottomOneConeOneCube() {
                List<PathPlannerTrajectory> bottomOneConeOneCubeLeave = PathPlanner.loadPathGroup(
                                "bottomOneConeOneCube",
                                new PathConstraints(3.5,
                                                2.75));
                // for (int i = 0; i < bottomOneConeOneCubeLeave.size(); i++) {
                // m_Drive.autoPath.getObject("traj" +
                // i).setTrajectory(bottomOneConeOneCubeLeave.get(i - 1));
                // }
                return autoBuilder.fullAuto(bottomOneConeOneCubeLeave);

        }

        public Command topOneConeOneCubeBalance() {
                List<PathPlannerTrajectory> topOneConeOneCubeBalance = PathPlanner.loadPathGroup(
                                "topOneConeOneCubeBalance",
                                new PathConstraints(3, 3));
                // for (int i = 0; i < topOneConeOneCubeBalance.size(); i++) {
                // m_Drive.autoPath.getObject("traj" +
                // i).setTrajectory(topOneConeOneCubeBalance.get(i - 1));
                // }
                return autoBuilder.fullAuto(topOneConeOneCubeBalance)
                                .andThen(new Balance(m_Drive));

        }

        public Command bottomOneConeOneCubeBalance() {
                List<PathPlannerTrajectory> bottomOneConeOneCubeBalance = PathPlanner.loadPathGroup(
                                "bottomOneConeOneCubeBalance",
                                new PathConstraints(3, 3));
                // for (int i = 0; i < bottomOneConeOneCubeBalance.size(); i++) {
                // m_Drive.autoPath.getObject("traj" +
                // i).setTrajectory(bottomOneConeOneCubeBalance.get(i - 1));
                // }
                return autoBuilder.fullAuto(bottomOneConeOneCubeBalance)
                                .andThen(new Balance(m_Drive));
        }

        public Command middleOneConeOneCubeBalanceBottom() {
                List<PathPlannerTrajectory> middleOneConeOneCubeBalanceBottom = PathPlanner.loadPathGroup(
                                "middleOneConeOneCubeBalanceBottom",
                                new PathConstraints(3, 3));
                // for (int i = 0; i < bottomOneConeOneCubeBalance.size(); i++) {
                // m_Drive.autoPath.getObject("traj" +
                // i).setTrajectory(bottomOneConeOneCubeBalance.get(i - 1));
                // }
                return autoBuilder.fullAuto(middleOneConeOneCubeBalanceBottom)
                                .andThen(new Balance(m_Drive));
        }

        // Three Game Piece Autos
        public Command topOneConeTwoCubeLow() {
                List<PathPlannerTrajectory> topOneConeTwoCubeLeave = PathPlanner.loadPathGroup(
                                "topOneConeTwoCubeLow",
                                new PathConstraints(4,
                                                3));
                // for (int i = 0; i < topOneConeOneCubeLeave.size(); i++) {
                // m_Drive.autoPath.getObject("traj" +
                // i).setTrajectory(topOneConeOneCubeLeave.get(i - 1));
                // }
                return autoBuilder.fullAuto(topOneConeTwoCubeLeave);

        }

        public Command topOneConeTwoCubeMid() {
                List<PathPlannerTrajectory> topOneConeTwoCubeLeave = PathPlanner.loadPathGroup(
                                "topOneConeTwoCubeMid",
                                new PathConstraints(4,
                                                3));
                // for (int i = 0; i < topOneConeOneCubeLeave.size(); i++) {
                // m_Drive.autoPath.getObject("traj" +
                // i).setTrajectory(topOneConeOneCubeLeave.get(i - 1));
                // }
                return autoBuilder.fullAuto(topOneConeTwoCubeLeave);

        }

        public Command topLowThreeCube() {
                List<PathPlannerTrajectory> topLowThreeCube = PathPlanner.loadPathGroup(
                                "topLowThreeCube",
                                new PathConstraints(1.75, 1.5));
                // for (int i = 0; i < topLowThreeCube.size(); i++) {
                // m_Drive.autoPath.getObject("traj" + i).setTrajectory(topLowThreeCube.get(i -
                // 1));
                // }
                return autoBuilder.fullAuto(topLowThreeCube);
        }

        public Command bottomLowThreeCube() {
                List<PathPlannerTrajectory> bottomLowThreeCube = PathPlanner.loadPathGroup(
                                "bottomLowThreeCube",
                                new PathConstraints(2, 2));
                // for (int i = 0; i < bottomLowThreeCube.size(); i++) {
                // m_Drive.autoPath.getObject("traj" + i).setTrajectory(bottomLowThreeCube.get(i
                // - 1));
                // }
                return autoBuilder.fullAuto(bottomLowThreeCube);
        }

        // TESTING
        public Command testSimultaneousMovement() {
                List<PathPlannerTrajectory> testSimultaneousMovement = PathPlanner.loadPathGroup(
                                "testSimultaneousMovement",
                                new PathConstraints(1, 1));
                // for (int i = 0; i < testSimultaneousMovement.size(); i++) {
                // m_Drive.autoPath.getObject("traj" +
                // i).setTrajectory(testSimultaneousMovement.get(i - 1));
                // }
                return autoBuilder.fullAuto(testSimultaneousMovement);

        }

}