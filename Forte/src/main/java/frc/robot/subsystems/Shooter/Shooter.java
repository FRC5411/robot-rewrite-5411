// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Shooter extends SubsystemBase {
  /** Creates a new Feeder. */
  private CANSparkMax pivotMotor;
  private DutyCycleEncoder pivotEncoder;
  private TalonFX topMotor;
  private TalonFX bottomMotor;
  
  public Shooter() {
    pivotMotor = new CANSparkMax(ShooterConstants.PIVOT_PORT, MotorType.kBrushless);
    topMotor = new TalonFX(ShooterConstants.UPPER_ROLLER_PORT, "drivetrain");
    bottomMotor = new TalonFX(ShooterConstants.LOWER_ROLLER_PORT, "drivetrain");
    pivotEncoder = new DutyCycleEncoder(ShooterConstants.PIVOT_ENCODER_PORT);

    config();
  }

  private void config(){
    pivotMotor.clearFaults();
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.burnFlash();
    pivotMotor.setSmartCurrentLimit((40));
    pivotMotor.setInverted(false);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    topMotor.clearStickyFaults();
    topMotor.setNeutralMode(NeutralModeValue.Coast);
    topMotor.setInverted(false);
    topMotor.getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit((25d)).withStatorCurrentLimit((20d)));

    bottomMotor.clearStickyFaults();
    bottomMotor.setNeutralMode(NeutralModeValue.Coast);
    bottomMotor.setInverted(false);
    bottomMotor.getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit((25d)).withStatorCurrentLimit((20d)));
  }

  public double getPivotAngle(){
    return (-pivotEncoder.getAbsolutePosition() * 360) + ShooterConstants.PIVOT_OFFSET;
  }

  public double getPivotAngleRadians(){
    return (getPivotAngle() * Math.PI / 180);
  }

  public double getPivotVelocity(){
    return pivotMotor.getEncoder().getVelocity();
  }

  public double getShooterVelocity(){
    return topMotor.getVelocity().getValueAsDouble();
  }

  //TODO: Check papi
  public double getPivotAppliedAmps(){
    return pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
  }

  public Command shooterPID(double setpoint) {
    ProfiledPIDController pivotController;
    ArmFeedforward pivotFeedforward;

    pivotController = new ProfiledPIDController(
      ShooterConstants.PIVOT_P,
      ShooterConstants.PIVOT_I,
      ShooterConstants.PIVOT_D, 
      new TrapezoidProfile.Constraints(600, 600));

    pivotFeedforward = new ArmFeedforward(0, 0.03, 0);

    return new FunctionalCommand(
      () -> {
        pivotController.reset(getPivotAngle());
      }, 
      () -> {
        pivotMotor.set(-pivotController.calculate(getPivotAngle(), setpoint));
        if(pivotController.atGoal()){
          pivotMotor.set(-pivotFeedforward.calculate(getPivotAngleRadians(), getPivotVelocity()));
        }
      }, 
      (interrupted) -> {

      }, 
      () -> pivotController.atGoal(), 
      this
      );
  }

  public void shoot(double topRoller, double bottomRoller){
    topMotor.set(topRoller);
    bottomMotor.set(bottomRoller);
  }
  
  public void motorZero(){
    topMotor.set(0);
    bottomMotor.set(0);
    pivotMotor.set(0);
  }

  public void setShooterVelocity(double demand){
    topMotor.setControl(new VelocityDutyCycle((demand / (60d))));
    bottomMotor.setControl(new VelocityDutyCycle((demand / (60d))));
  }

  public Command shootRPMAmp(){
    return new InstantCommand(() -> shoot((0), (0.21)));
  }

  public Command shootRPMFeed(){
    return new InstantCommand(() -> shoot((0.45), (0.45)));
  }

  public Command shootRPMSubwoofer(){
    return new InstantCommand(() -> shoot((0.5), (0.5)));
  }

  public Command shootRPMLaser(){
    return new InstantCommand(() -> shoot((0.8), (0.8)));
  }

  public Command shootRPMWing(){
    return new InstantCommand(() -> shoot((0.7), (0.7)));
  }

  public Command shootRPMLob(){
    return new InstantCommand(() -> shoot((0.3), (0.3)));
  }

  public Command reverseShoot(){
    return new InstantCommand(() -> shoot((0.2), (0.2)));
  }

  public Command zeroShoot(){
    return new InstantCommand(() -> motorZero());
  }

  public Command movePivotSlowUp(){
    return new InstantCommand(() -> pivotMotor.set((0.3)));
  }

  public Command movePivotSlowDown(){
    return new InstantCommand(() -> pivotMotor.set(-0.3));
  }

  public Command movePivotZero(){
    return new InstantCommand(() -> pivotMotor.set((0)));
  }

  public Command setShooterRPM(double demand){
    return new InstantCommand(() -> setShooterVelocity(demand));
  }

  public Command shooterIntake(){
    return shooterPID(ShooterConstants.PIVOT_INTAKE_ANGLE);
  }

  public Command shooterAmp(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_AMP_ANGLE),
      shootRPMAmp()
    );
  }

  public Command shooterWing(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_WING_ANGLE),
      shootRPMWing()
    );
  }

  public Command shooterIdle(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_IDLE_ANGLE),
      zeroShoot()
    );
  }

  public Command shooterSubwoofer(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_SUBWOOFER_ANGLE),
      shootRPMSubwoofer()
    );
  }

  public Command shooterFeed(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_SUBWOOFER_ANGLE),
      shootRPMFeed()
    );
  }

  public Command shooterLaser(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_IDLE_ANGLE),
      shootRPMLaser()
    );
  }

  public Command shooterLob(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_SUBWOOFER_ANGLE),
      shootRPMLob()
    );
  }

  public Command shooterPodium(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_PODIUM_ANGLE),
      shootRPMSubwoofer()
    );
  }

  
  @Override
   public void periodic() {
    SmartDashboard.putNumber("Shooter RPM", getShooterVelocity());
    SmartDashboard.putNumber("Shooter Angle", getPivotAngle());
    SmartDashboard.putNumber("Shooter Temperature", getPivotAppliedAmps());
   }
}
