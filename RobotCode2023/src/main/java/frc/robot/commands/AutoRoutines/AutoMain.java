package frc.robot.commands.AutoRoutines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.gameplay.automations.Balance;
import frc.robot.commands.gameplay.automations.armTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

public class AutoMain extends CommandBase {

    SwerveDrive m_Drive;
    Arm m_Arm;
    Intake m_Intake;

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
    }

    // Base Commands
    public final Command scoreHigh() {
        return (new armTrajectory(Constants.armConstants.HIGH_POSITION, m_Arm)
                .andThen(new WaitCommand(1.5))
                .andThen(new RunCommand(m_Intake::runOutake, m_Intake).withTimeout(1))
                .andThen(new InstantCommand(m_Intake::intakeOff, m_Intake))
                .andThen(new armTrajectory(Constants.armConstants.DEFAULT_POSITION, m_Arm)));

    }

    public final Command scoreMiddle() {
        return (new armTrajectory(Constants.armConstants.MID_POSITION, m_Arm)
                .andThen(new RunCommand(m_Intake::runOutake, m_Intake).withTimeout(1))
                .andThen(new InstantCommand(m_Intake::intakeOff, m_Intake))
                .andThen(new armTrajectory(Constants.armConstants.DEFAULT_POSITION, m_Arm)));
    }

    public final Command scoreLow() {
        return (new armTrajectory(Constants.armConstants.LOW_UPRIGHT_CONE_POSITION, m_Arm)
                .andThen(new RunCommand(m_Intake::runOutake, m_Intake).withTimeout(1))
                .andThen(new InstantCommand(m_Intake::intakeOff, m_Intake))
                .andThen(new armTrajectory(Constants.armConstants.DEFAULT_POSITION, m_Arm)));
    }

    public final Command balance() {
        return (new Balance(m_Drive));
    }

    public Command topOneConeLeaveCommand() {
        PathPlannerTrajectory topOneConeLeave = PathPlanner.loadPath("topOneConeLeave", new PathConstraints(3, 1.5));

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

    public Command middleOneConeLeaveCommand() {
        PathPlannerTrajectory middleOneConeLeave = PathPlanner.loadPath("middleOneConeLeave",
                new PathConstraints(1.5, 0.75));

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
        PathPlannerTrajectory BottomOneConeLeave = PathPlanner.loadPath("bottomOneConeLeave",
                new PathConstraints(3, 1.5));

        return scoreHigh()
                .andThen(new FollowPathWithEvents(
                        m_Drive.followTrajectoryCommand(BottomOneConeLeave, true),
                        BottomOneConeLeave.getMarkers(),
                        Constants.AutoConstants.eventMap));

    }

    public Command bottomOneConeBalanceCommand() {
        PathPlannerTrajectory bottomOneConeBalance = PathPlanner.loadPath("bottomOneConeBalance",
                new PathConstraints(3, 1.5));

        return scoreHigh()
                .andThen(new FollowPathWithEvents(
                        m_Drive.followTrajectoryCommand(bottomOneConeBalance, true),
                        bottomOneConeBalance.getMarkers(),
                        Constants.AutoConstants.eventMap)
                        .andThen(new Balance(m_Drive)));

    }

}