package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.autocommands.DriveDistance;
import frc.robot.commands.autocommands.FeedOff;
import frc.robot.commands.autocommands.FeedOn;
import frc.robot.commands.autocommands.FullTransport;
import frc.robot.commands.autocommands.IntakeOn;
import frc.robot.commands.autocommands.IntakeUpLimited;
import frc.robot.commands.autocommands.ShooterDownLimited;
import frc.robot.commands.autocommands.ShooterOff;
import frc.robot.commands.autocommands.ShooterOn;
import frc.robot.commands.autocommands.ShooterToPosition;
import frc.robot.commands.autocommands.TimedDistance;
import frc.robot.commands.autocommands.TimedIntakeDown;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ThreePieceAuto extends SequentialCommandGroup {
    public ThreePieceAuto(Swerve s_Swerve, Intake intake, Shooter shooter){
        double distance = -1/.71; // Through experimentation we found that the robot only travels 71% of the desired distance
        double sideDistance = 1.4/.71;
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);



                Trajectory forwardTrajectory =
                TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, new Rotation2d(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(-1*distance/2, 0)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(-1*distance, 0, new Rotation2d(0)),
                    config);
              
                config.setReversed(true);
    
            // An example trajectory to follow.  All units in meters.
            Trajectory backTrajectory =
                TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, new Rotation2d(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(distance/2, 0)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(distance, 0, new Rotation2d(0)),
                    config);

            Trajectory leftTrajectory =
                TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, new Rotation2d(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(distance/2.5, sideDistance/2)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(distance/2.5, sideDistance, new Rotation2d(0)),
                    config);

            Trajectory rightTrajectory =
                TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, new Rotation2d(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(-distance/6, -1*sideDistance/2)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(-distance/5, -1*sideDistance, new Rotation2d(0)),
                    config);

            
          
          

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, Constants.AutoConstants.kIThetaController, Constants.AutoConstants.kDThetaController, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goBack =
            new SwerveControllerCommand(
                backTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, Constants.AutoConstants.kDXController),
                new PIDController(Constants.AutoConstants.kPYController, Constants.AutoConstants.kIYController, Constants.AutoConstants.kDYController),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

          SwerveControllerCommand goBack2 =
            new SwerveControllerCommand(
                backTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, Constants.AutoConstants.kDXController),
                new PIDController(Constants.AutoConstants.kPYController, Constants.AutoConstants.kIYController, Constants.AutoConstants.kDYController),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

          SwerveControllerCommand goForward =
            new SwerveControllerCommand(
                forwardTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, Constants.AutoConstants.kDXController),
                new PIDController(Constants.AutoConstants.kPYController, Constants.AutoConstants.kIYController, Constants.AutoConstants.kDYController),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

            SwerveControllerCommand goForward2 =
            new SwerveControllerCommand(
                forwardTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, Constants.AutoConstants.kDXController),
                new PIDController(Constants.AutoConstants.kPYController, Constants.AutoConstants.kIYController, Constants.AutoConstants.kDYController),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

            SwerveControllerCommand goRight =
            new SwerveControllerCommand(
                rightTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, Constants.AutoConstants.kDXController),
                new PIDController(Constants.AutoConstants.kPYController, Constants.AutoConstants.kIYController, Constants.AutoConstants.kDYController),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

            SwerveControllerCommand goLeft =
            new SwerveControllerCommand(
                leftTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, Constants.AutoConstants.kDXController),
                new PIDController(Constants.AutoConstants.kPYController, Constants.AutoConstants.kIYController, Constants.AutoConstants.kDYController),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new ShooterOn(shooter),
            new WaitCommand(1),
            new FeedOn(shooter),
            new WaitCommand(1),
            new ShooterOff(shooter),
            new FeedOff(shooter),
            new TimedIntakeDown(intake, 0.7),
            new InstantCommand(() -> s_Swerve.setOdometry(backTrajectory.getInitialPose())),
            new ParallelCommandGroup(new IntakeOn(intake), goBack, new ShooterDownLimited(shooter)),
            new InstantCommand(() -> s_Swerve.setOdometry(forwardTrajectory.getInitialPose())),
            new ParallelCommandGroup(goForward, new FullTransport(intake, shooter)),
            new ShooterOn(shooter),
            new ShooterToPosition(shooter, Constants.Shooter.speakerPosition),
            new FeedOn(shooter),
            new WaitCommand(1),
            new ShooterOff(shooter),
            new FeedOff(shooter),
            new InstantCommand(() -> s_Swerve.setOdometry(leftTrajectory.getInitialPose())),
            goLeft, 
            new TimedIntakeDown(intake, 0.3),
            new InstantCommand(() -> s_Swerve.setOdometry(backTrajectory.getInitialPose())),
            new ParallelCommandGroup(new IntakeOn(intake), goBack2, new ShooterDownLimited(shooter)),
            new InstantCommand(() -> s_Swerve.setOdometry(forwardTrajectory.getInitialPose())),
            new ParallelCommandGroup(goForward2, new FullTransport(intake, shooter)),
            new InstantCommand(() -> s_Swerve.setOdometry(rightTrajectory.getInitialPose())),
            new ShooterOn(shooter),
            new ParallelCommandGroup(goRight, new ShooterToPosition(shooter, Constants.Shooter.speakerPosition)),
            new WaitCommand(0.3),
            new FeedOn(shooter),
            new WaitCommand(1),
            new ShooterOff(shooter),
            new FeedOff(shooter)
        );
    }
}

     