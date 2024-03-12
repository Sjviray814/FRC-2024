// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private CANSparkMax leftIntakeMotor, rightIntakeMotor, intakeArticulatorMotor;
  private IdleMode intakeIdleMode, intakeArticulatorIdleMode;
  private DigitalInput beamBrake;
  private RelativeEncoder intakeEncoder;
  private SparkPIDController intakePIDController;

  private final double upPosition = 0;
  private final double downPosition = 0;


  /** Creates a new Intake. */
  public Intake() {

    leftIntakeMotor = new CANSparkMax(Constants.Intake.leftIntakeID, MotorType.kBrushless);
    rightIntakeMotor = new CANSparkMax(Constants.Intake.rightIntakeID, MotorType.kBrushless);
    intakeArticulatorMotor = new CANSparkMax(Constants.Intake.articulateIntakeID, MotorType.kBrushless);

    beamBrake = new DigitalInput(3);

    // Configure Idle Modes:
    intakeIdleMode = IdleMode.kBrake;
    intakeArticulatorIdleMode = IdleMode.kBrake;

    // Get encoders:
    initPID();

  }

  public void configureMotors(){
    // Restore Factory Defaults:
    leftIntakeMotor.restoreFactoryDefaults();
    rightIntakeMotor.restoreFactoryDefaults();
    intakeArticulatorMotor.restoreFactoryDefaults();

    // Set Idle Modes:
    rightIntakeMotor.setIdleMode(intakeIdleMode);
    leftIntakeMotor.setIdleMode(intakeIdleMode);
    intakeArticulatorMotor.setIdleMode(intakeArticulatorIdleMode);

    leftIntakeMotor.setInverted(true);
  }

  public void initPID(){
    intakeEncoder = intakeArticulatorMotor.getEncoder();
    intakePIDController = intakeArticulatorMotor.getPIDController();

    intakePIDController.setP(Constants.Intake.intakePP, Constants.Intake.intakeSlot);
    intakePIDController.setI(Constants.Intake.intakePI, Constants.Intake.intakeSlot);
    intakePIDController.setD(Constants.Intake.intakePD, Constants.Intake.intakeSlot);
    intakePIDController.setFF(Constants.Intake.intakePF, Constants.Intake.intakeSlot);

    intakeArticulatorMotor.burnFlash();
  }

  public void resetIntakeEncoders(){
    intakeEncoder.setPosition(0);
  }

  public double getIntakeEncoder(){
    return intakeEncoder.getPosition();
  }

  public void setIntakePosition(double position){
    intakePIDController.setReference(position, ControlType.kPosition, 0);
  }

  public void intakeOn(){
    leftIntakeMotor.set(-.25);
    rightIntakeMotor.set(-.25);
  }

  public boolean getBeamBrake(){
    return beamBrake.get();
  }

  public void intakeOff(){
    intakeArticulatorMotor.set(0);
  }

  public void intakeReverse(){
    leftIntakeMotor.set(.25);
    rightIntakeMotor.set(.25);
  }

  public void intakeUp(){
    intakeArticulatorMotor.set(.2);
  }
  
  public void intakeDown(){
    intakeArticulatorMotor.set(-.2);
  }

  public void intakeStop(){
    leftIntakeMotor.set(0);
    rightIntakeMotor.set(0);
  }

  public void intakePIDUp(){
    setIntakePosition(upPosition);
  }

  public void intakePIDDown(){
    setIntakePosition(downPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Beam Break", getBeamBrake());
    SmartDashboard.putNumber("intakePosition", getIntakeEncoder());
  }
}
