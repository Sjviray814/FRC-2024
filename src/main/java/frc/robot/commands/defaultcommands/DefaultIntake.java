// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class DefaultIntake extends Command {
  /** Creates a new DefaultIntake. */
  private Intake intake;
  private BooleanSupplier intakeUp, intakeDown, intakeOn;

  public DefaultIntake(BooleanSupplier intakeUp, BooleanSupplier intakeDown, BooleanSupplier intakeOn, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.intakeUp = intakeUp;
    this.intakeDown = intakeDown;

    addRequirements(intake);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeOn.getAsBoolean()){
      intake.intakeOn();
    }
    else{
      intake.intakeStop();
    }

    if(intakeUp.getAsBoolean()){
      intake.intakeUp();
    }
    else if(intakeDown.getAsBoolean()){
      intake.intakeDown();
    }
    else{
      intake.intakeOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
