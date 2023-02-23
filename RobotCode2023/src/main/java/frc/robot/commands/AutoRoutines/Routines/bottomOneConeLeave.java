package frc.robot.commands.AutoRoutines.Routines;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.gameplay.automations.DriveTo;
import frc.robot.commands.gameplay.automations.armTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class bottomOneConeLeave extends SequentialCommandGroup{

    

    public bottomOneConeLeave(SwerveDrive m_Drive, Arm m_Arm)    {

       addCommands(new armTrajectory(Constants.armConstants.HIGH_POSITION, m_Arm));
        // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    PathPlannerTrajectory bottomOneConeLeave = PathPlanner.loadPath("bottom1ConeLeave", new PathConstraints(4, 3));

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    

    FollowPathWithEvents command = new FollowPathWithEvents(
    m_Drive.followTrajectoryCommand(bottomOneConeLeave, true),
    bottomOneConeLeave.getMarkers(),
    eventMap
);

    }
    
}
