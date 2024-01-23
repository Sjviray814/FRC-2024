// package frc.robot.autos;

// import frc.robot.Constants;
// import frc.robot.commands.autocommands.AutoDrive;
// import frc.robot.commands.autocommands.AutoTurn;
// import frc.robot.subsystems.Swerve;

// import java.io.IOException;
// import java.nio.file.Path;
// import java.time.InstantSource;
// import java.util.ArrayList;
// import java.util.HashMap;
// import java.util.List;

// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.FollowPathWithEvents;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;


// public class DriveForward extends SequentialCommandGroup {

//     public DriveForward(Swerve swerve){

        

//         PathPlannerPath dF = PathPlannerPath.fromPathFile("driveBackward");
//         //, new PathConstraints(4, 4)
        
//         // PathPlannerPath.transformTrajectoryForAlliance(dF, DriverStation.getAlliance());

//         // Command driveForwardCommand = swerve.followPathCommand(dF);

//         SmartDashboard.putNumber("Initial Rotation", dF.getPreviewStartingHolonomicPose().getRotation().getDegrees());
        
        

        
//         addCommands(

//             // new InstantCommand(() -> swerve.zeroGyro()),
//             // new InstantCommand(() -> swerve.setOdometry(new Pose2d(dF.getPreviewStartingHolonomicPose().getTranslation(), dF.getPreviewStartingHolonomicPose().getRotation()))),
//             // driveForwardCommand
//             // new AutoDrive(10, swerve, true, false)

//         );

//     }
// }

