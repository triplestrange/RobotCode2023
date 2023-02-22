package frc.robot.commands.AutoRoutines.Routines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.gameplay.automations.DriveTo;
import frc.robot.commands.gameplay.automations.armTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class OneConeLeaveCommunity extends SequentialCommandGroup{

    

    public OneConeLeaveCommunity(SwerveDrive m_Drive, Arm m_Arm)    {

        PathPlannerTrajectory OneConeLeaveCommunity = PathPlanner.loadPath("OneCone, Leave Community", new PathConstraints(4, 3));

        addCommands(m_Drive.followTrajectoryCommand(OneConeLeaveCommunity, true));

    }
    
}
