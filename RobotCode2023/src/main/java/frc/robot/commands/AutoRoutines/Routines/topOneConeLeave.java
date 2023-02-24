package frc.robot.commands.AutoRoutines.Routines;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.gameplay.automations.armPositions;
import frc.robot.commands.gameplay.automations.armTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class topOneConeLeave extends SequentialCommandGroup{

    

    public topOneConeLeave(SwerveDrive m_Drive, Arm m_Arm)    {

       addCommands(new armTrajectory(Constants.armConstants.HIGH_POSITION, m_Arm));
        // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    PathPlannerTrajectory topOneConeLeave = PathPlanner.loadPath("top1ConeLeave", new PathConstraints(4, 3));

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
        

    FollowPathWithEvents command = new FollowPathWithEvents(
    m_Drive.followTrajectoryCommand(topOneConeLeave, true),
    topOneConeLeave.getMarkers(),
    Constants.AutoConstants.eventMap
);

    }
    
}
