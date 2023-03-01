package frc.robot.commands.AutoRoutines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.gameplay.automations.armTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

public class TopOneConeLeave extends SequentialCommandGroup{

    

    public TopOneConeLeave(SwerveDrive m_Drive, Arm m_Arm, Intake m_Intake)    {
        PathPlannerTrajectory TopOneConeLeave = PathPlanner.loadPath("top1ConeLeave", new PathConstraints(1, 0.2));

        addCommands(new armTrajectory(Constants.armConstants.HIGH_POSITION, m_Arm));
        addCommands(new RunCommand(m_Intake::runOutake, m_Intake).withTimeout(3));
        addCommands(new armTrajectory(Constants.armConstants.DEFAULT_POSITION, m_Arm));

        addCommands(new FollowPathWithEvents(
        m_Drive.followTrajectoryCommand(TopOneConeLeave, true),
        TopOneConeLeave.getMarkers(),
        Constants.AutoConstants.eventMap
    ));

    }
    
}