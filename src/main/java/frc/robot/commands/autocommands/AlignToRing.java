package frc.robot.commands.autocommands;

import java.lang.invoke.ConstantCallSite;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;



public class AlignToRing extends Command{
  private Swerve swerve;
  private Timer timer;
  private double rotation, savedRotation;
  
  private PhotonCamera camera = new PhotonCamera("camera1");

  
  public AlignToRing(Swerve swerve) {
    timer = new Timer();
    this.swerve = swerve;
    this.rotation = 0;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.''
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // rotation = -Limelight.getTx(); 

    var result = camera.getLatestResult();

    if(!result.hasTargets()){
      rotation = 0;
    }
    else{
      PhotonTrackedTarget target = result.getBestTarget();
      rotation = target.getYaw();
    }
    SmartDashboard.putNumber("Ring Angle Rotation", rotation);
    
      if(rotation != 0){
        savedRotation = rotation;
      }

      // double maxSpeed = Math.abs(savedRotation)/35.0 + 0.02;
      // if(maxSpeed > 0.7) maxSpeed = 0.7;

      //   if(Math.abs(savedRotation) >= 12){
      //       swerve.driveSlow(new Translation2d(0, 0), savedRotation, true, true, 0.5);
      //   }
      //   else if(Math.abs(savedRotation) > 2){
      //       swerve.driveSlow(new Translation2d(0, 0), savedRotation, true, true, 0.1);
      //   }
      //   else{
      //       swerve.driveSlow(new Translation2d(0, 0), savedRotation, true, true, 0.05);
      //   }
        // swerve.driveSlow(new Translation2d(0,0), savedRotation, true, true, maxSpeed);
        
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    swerve.drive(new Translation2d(), 0, true, true);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((rotation != 0 && Math.abs(rotation) < 0.1) || timer.hasElapsed(3)){
      return true;
    } else {
      return false;
    }
    // // return (Math.abs(rotation - swerve.getPose().getRotation().getDegrees()) <= 5 || timer.hasElapsed(3));
  }
}
