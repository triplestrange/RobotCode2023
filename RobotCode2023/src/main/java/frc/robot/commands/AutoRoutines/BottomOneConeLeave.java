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

public class BottomOneConeLeave extends SequentialCommandGroup{

    

    public BottomOneConeLeave(SwerveDrive m_Drive, Arm m_Arm, Intake m_Intake)    {
        PathPlannerTrajectory BottomOneConeLeave = PathPlanner.loadPath("bottom1ConeLeave", new PathConstraints(1, 0.2));

        addCommands(new armTrajectory(Constants.armConstants.HIGH_POSITION, m_Arm));
        addCommands(new RunCommand(m_Intake::runOutake, m_Intake).withTimeout(3));
        addCommands(new armTrajectory(Constants.armConstants.DEFAULT_POSITION, m_Arm));
        
        addCommands(new FollowPathWithEvents(
        m_Drive.followTrajectoryCommand(BottomOneConeLeave, true),
        BottomOneConeLeave.getMarkers(),
        Constants.AutoConstants.eventMap
    ));

    }
    
}
