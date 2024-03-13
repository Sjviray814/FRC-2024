// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TimedDistance extends Command {
  /** Creates a new TimedTrajectory. */
  private Swerve swerve;
  private double seconds, distance;
  private Timer timer;
  public TimedDistance(Swerve swerve, double seconds, double distance) {
    this.swerve = swerve;
    this.seconds = seconds;
    this.distance = distance;
    this.timer = new Timer();
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new DriveDistance(swerve, distance);
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
    return timer.hasElapsed(seconds);
  }
}

