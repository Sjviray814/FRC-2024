// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeOnUnhindered extends Command {
  /** Creates a new TimedDriveOut. */
  private Intake intake;
  private Timer timer;
  
  public IntakeOnUnhindered(Intake intake) {
    this.intake = intake;
    this.timer = new Timer();

    addRequirements(intake);
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
    intake.intakeOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intake.intakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
