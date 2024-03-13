// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;



public class IntakeToAngle extends Command {
  /** Creates a new NeckToLength Command */

  private Intake intake;
  private double angle;

  public IntakeToAngle(Intake intake, double angle) {
    this.intake = intake;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakePosition(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(angle - intake.getIntakeEncoder()) <= .1);    

    //4 is the radius of the interior of the cone
    //4 is a buffer zone for the robot -> 2 inches before, 2 inches past = 4 total inches
  }
}
