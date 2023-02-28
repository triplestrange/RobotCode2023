package frc.robot.commands.gameplay.automations;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.JointAngles;

public class armTrajectory extends CommandBase{
    private final Arm arm;
    private final Pose2d targetPose2d;
    private Trajectory trajectory;
    private final Timer timer = new Timer();
    private double initialWristAngle;
    private double finalWristAngle;
    private Boolean exception; 
    public armTrajectory(Pose2d targetPose2d, Arm arm) {
        addRequirements(arm);
        this.arm = arm;
        this.targetPose2d = targetPose2d;
        this.finalWristAngle = targetPose2d.getRotation().getRadians();
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        Translation2d currentPos  = arm.getArmPosition();
        double initialAngle;
        double endingAngle;
        exception = false;
        initialWristAngle = arm.getWrist();
        if (currentPos.getX() < (20 * 0.0254))    {
            initialAngle = Math.toRadians(90); 
            SmartDashboard.putNumber("initialAngle", initialAngle);
        }

        else if (currentPos.getY() < (7 * 0.0254)) {
            initialAngle = Math.toRadians(75);
            SmartDashboard.putNumber("initialAngle", initialAngle);
        }

        else {

            initialAngle = Math.PI; 
            SmartDashboard.putNumber("initialAngle", initialAngle);
        }

        if (targetPose2d.getX() < (20 * 0.0254))    {
            endingAngle = Math.toRadians(270); 
            SmartDashboard.putNumber("endingAngle", endingAngle);
        }

        else if (targetPose2d.getY() < (7 * 0.0254)) {
            endingAngle = Math.toRadians(255);
            SmartDashboard.putNumber("endingAngle", endingAngle);
        }

        else {

            endingAngle = 0; 
            SmartDashboard.putNumber("endingAngle", endingAngle
            );
        }

        try {trajectory =
        TrajectoryGenerator.generateTrajectory(
            
            // Start at the origin facing the +X direction
            new Pose2d(currentPos, new Rotation2d(initialAngle)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(targetPose2d.getX(), targetPose2d.getY(), new Rotation2d(endingAngle)),
            Constants.armConstants.config);}
        catch (TrajectoryGenerationException trajExcep) {

            exception = true;
        }
        
        timer.reset();
        timer.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!exception)  {
            Trajectory.State targetState = trajectory.sample(timer.get());
        
        SmartDashboard.putNumber("armTrajX", targetState.poseMeters.getX());
        SmartDashboard.putNumber("armTrajY", targetState.poseMeters.getY());


        JointAngles targetAngles = JointAngles.anglesFrom2D(targetState.poseMeters.getX(), targetState.poseMeters.getY(), finalWristAngle);
        Rotation2d direction = targetState.poseMeters.getRotation();
        Vector<N2> linearVelocity = VecBuilder.fill(direction.getCos(), direction.getSin()).times(targetState.velocityMetersPerSecond);
         
        targetAngles.shoulderAngle = MathUtil.clamp(targetAngles.shoulderAngle, Math.toRadians(45), Math.toRadians(97));
        targetAngles.elbowAngle = MathUtil.clamp(targetAngles.elbowAngle, Math.toRadians(-80), Math.toRadians(45));
        Vector <N2> angleVector = arm.angleVelocity(linearVelocity);
        arm.setShoulder(targetAngles.shoulderAngle, 1 * angleVector.get(0, 0));
        arm.setElbow(targetAngles.elbowAngle, 1 * angleVector.get(1, 0));
        arm.setWrist(MathUtil.clamp(initialWristAngle + (finalWristAngle - initialWristAngle) * timer.get()/trajectory.getTotalTimeSeconds(), -(Math.PI/2), Math.PI/2), 
        finalWristAngle - initialWristAngle / trajectory.getTotalTimeSeconds());
        }
        
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

      if (trajectory.getTotalTimeSeconds() < timer.get() || exception)    {

        return true;
      }

      return false;
    }
  
  
  }

