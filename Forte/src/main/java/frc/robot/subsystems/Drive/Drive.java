// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Constants.Mode;

/** Swerve drive */
public class Drive extends SubsystemBase {
  public static final double TRACK_WIDTH_X_M = Units.inchesToMeters(29.5);
  public static final double TRACK_WIDTH_Y_M = Units.inchesToMeters(29.5);
  public static final double DRIVEBASE_RADIUS_M =
      Math.hypot(TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0);
  public static final double MAX_LINEAR_SPEED_MPS = Units.feetToMeters(19.1);
  public static final double MAX_ANGULAR_SPEED_MPS = MAX_LINEAR_SPEED_MPS / DRIVEBASE_RADIUS_M;
  // Second argument is the max accel
  public static final ModuleLimits MODULE_LIMITS =
      new ModuleLimits(MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS * 5, MAX_ANGULAR_SPEED_MPS);

  private final Translation2d[] MODULE_TRANSLATIONS = getModuleTranslations();
  private final SwerveDriveKinematics KINEMATICS = getKinematics();
  private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

  // private SwerveSetpoint currentSetpoint =
  //     new SwerveSetpoint(
  //         new ChassisSpeeds(),
  //         new SwerveModuleState[] {
  //           new SwerveModuleState(),
  //           new SwerveModuleState(),
  //           new SwerveModuleState(),
  //           new SwerveModuleState()
  //         });
  // private SwerveSetpointGenerator setpointGenerator =
  //     new SwerveSetpointGenerator(KINEMATICS, MODULE_TRANSLATIONS);
  private boolean areModulesOrienting = false;

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();

  private Module[] modules = new Module[4]; // FL FR BL BR

  private Pose2d currentPose = new Pose2d();
  private Pose2d filteredPose = new Pose2d();

  // Used to compare pose estimator and odometry
  private SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(KINEMATICS, getRotation(), getModulePositions());

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(KINEMATICS, getRotation(), getModulePositions(), currentPose);

  private PIDConstants translationPathplannerConstants = new PIDConstants(1.25, 0.0, 0.0);
  private PIDConstants rotationPathplannerConstants = new PIDConstants(1.25, 0.0, 0.0);
  private boolean PProtationTargetOverride = false;

  private LinearFilter xFilter = LinearFilter.movingAverage(5);
  private LinearFilter yFilter = LinearFilter.movingAverage(5);

  private Field2d field = new Field2d();

  /** Creates a new swerve Drive. */
  public Drive(
      ModuleIO moduleFL, ModuleIO moduleFR, ModuleIO moduleBL, ModuleIO moduleBR, GyroIO gyro) {
    modules[0] = new Module(moduleFL, 0);
    modules[1] = new Module(moduleFR, 1);
    modules[2] = new Module(moduleBL, 2);
    modules[3] = new Module(moduleBR, 3);
    gyroIO = gyro;

    // Configure setpoint generator
    // setpointGenerator =
    //     SwerveSetpointGenerator.builder()
    //         .kinematics(KINEMATICS)
    //         .moduleLocations(MODULE_TRANSLATIONS)
    //         .build();

    // Configure PathPlanner
    AutoBuilder.configureHolonomic(
        () -> odometry.getPoseMeters(),
        this::setPose,
        this::getRobotRelativeSpeeds,
        this::runSwerve,
        new HolonomicPathFollowerConfig(
            translationPathplannerConstants,
            rotationPathplannerConstants,
            MAX_LINEAR_SPEED_MPS,
            DRIVEBASE_RADIUS_M,
            new ReplanningConfig(true, true)),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Drive/Odometry/Trajectory",
              activePath.toArray(new Pose2d[activePath.size()])); // Autolog the trajectory
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput(
              "Drive/Odometry/TrajectorySetpoint", targetPose); // Auto log the target setpoint
        });

    SmartDashboard.putData(field);
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("FrontLeft Module", modules[0].getAbsoluteAngle());
    // Logger.recordOutput("FrontRight Module", modules[1].getAbsoluteAngle());
    // Logger.recordOutput("BackLeft Module", modules[2].getAbsoluteAngle());
    // Logger.recordOutput("BackRight Module", modules[3].getAbsoluteAngle());

    gyroIO.updateInputs(gyroIOInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    Logger.processInputs("Drive/Gyro/Inputs", gyroIOInputs);

    for (var module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      // Log empty states
      Logger.recordOutput("Drive/Swerve/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/Swerve/SetpointsOptimized", new SwerveModuleState[] {});
    }
    if (DriverStation.isEStopped()) {
      for (var module : modules) {
        module.stop();
        module.setBrake(true); // Apply brakes if E-Stopped
      }
    }

    if (gyroIOInputs.connected) {
      poseEstimator.update(getRotation(), getModulePositions());
      odometry.update(getRotation(), getModulePositions());
    } else {
      poseEstimator.update(
          Rotation2d.fromDegrees(
              (poseEstimator.getEstimatedPosition().getRotation().getDegrees()
                      + Math.toRadians(getChassisSpeeds().omegaRadiansPerSecond) * 0.02)
                  % 360.0),
          getModulePositions());
      odometry.update(
          Rotation2d.fromDegrees(
              (odometry.getPoseMeters().getRotation().getDegrees()
                      + Math.toRadians(getChassisSpeeds().omegaRadiansPerSecond) * 0.02)
                  % 360.0),
          getModulePositions());
    }

    currentPose = poseEstimator.getEstimatedPosition();

    filteredPose =
        new Pose2d(
            xFilter.calculate(getPoseEstimate().getX()),
            yFilter.calculate(getPoseEstimate().getY()),
            getPoseEstimate().getRotation());

    field.setRobotPose(getFilteredPose());
  }

  /** Runs the swerve drive based on speeds */
  public void runSwerve(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = discretize(speeds); // Translational skew compensation
    desiredChassisSpeeds = discreteSpeeds;
    SwerveModuleState[] setpointStates = KINEMATICS.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, MAX_LINEAR_SPEED_MPS); // Normalize speeds

    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        optimizedSetpointStates[i] =
            modules[i].setDesiredState(
                setpointStates[i]); // setDesiredState returns the optimized state
      }
    }

    // Logger.recordOutput("Drive/Swerve/Setpoints", setpointStates);
    // Logger.recordOutput("Drive/Swerve/SetpointsOptimized", optimizedSetpointStates);
  


  /** Custom method for discretizing swerve speeds */
  private ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose =
        new Pose2d(
            speeds.vxMetersPerSecond * dt,
            speeds.vyMetersPerSecond * dt,
            new Rotation2d(speeds.omegaRadiansPerSecond * dt * 3));
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  /** Stops the drive */
  public void stop() {
    runSwerve(new ChassisSpeeds());
  }

  /** Reset the robot's pose */
  public void resetPose() {
    setPose(currentPose);
  }

  /** Reset the gyro heading */
  public void resetGyro() {
    gyroIO.resetGyro();
  }

  /** Reset the swerve modules */
  public void resetModules() {
    for (int i = 0; i < 4; i++) {
      modules[i].reset();
    }
  }

  /** Set the pose of the robot */
  public void setPose(Pose2d pose) {
    if (Constants.currentMode == Mode.SIM) {
      poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
      odometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
    } else {
      poseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
      odometry.resetPosition(getRotation(), getModulePositions(), pose);
    }

    currentPose = poseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()),
        getRotation());
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
}

  /** Add a vision measurement for the poseEstimator */
  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampS, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampS, stdDevs);
  }

  /** Returns PathFinder constraints */
  public PathConstraints getPathConstraints() {
    return new PathConstraints(
        3.0, 3.0, Units.degreesToRadians(540.0), Units.degreesToRadians(720.0));
  }

  /** Returns the drive's measured state (module azimuth angles and drive velocities) */
  @AutoLogOutput(key = "Drive/Swerve/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getModuleState();
    }

    return states;
  }

  /** Returns the swerve module's positions */
  @AutoLogOutput(key = "Drive/Swerve/ModulePositions")
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] =
          modules[i] != null ? modules[i].getModulePosition() : new SwerveModulePosition();
    }

    return positions;
  }

  /** Returns the pose of the robot with vision */
  @AutoLogOutput(key = "Drive/Odometry/PoseEstimate")
  public Pose2d getPoseEstimate() {
    return currentPose;
  }

  /** Returns the pose of the robot from odometer */
  @AutoLogOutput(key = "Drive/Odometry/DrivePose")
  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  /** Returns the filter pose of the robot */
  @AutoLogOutput(key = "Drive/Odometry/FilteredPose")
  public Pose2d getFilteredPose() {
    return filteredPose;
  }

  /** Returns the rotation of the robot */
  @AutoLogOutput(key = "Drive/Odometry/Rotation")
  public Rotation2d getRotation() {
    return gyroIOInputs.yawPosition;
  }

  /** Returns the maximum allowed linear (translational) speed */
  public double getMaxLinearSpeedMPS() {
    return MAX_LINEAR_SPEED_MPS;
  }

  /** Returns the maximum allowed rotational speed */
  public double getMaxAngularSpeedMPS() {
    return MAX_ANGULAR_SPEED_MPS;
  }

  /** Returns the current chassis speeds of th erobot */
  public ChassisSpeeds getChassisSpeeds() {
    return KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  /** Returns the current desired chassis speeds of the robot */
  public ChassisSpeeds getDesiredChassisSpeeds() {
    return desiredChassisSpeeds;
  }


  /** Returns the positions of the modules on the drive */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0),
      new Translation2d(TRACK_WIDTH_X_M / 2.0, -TRACK_WIDTH_Y_M / 2.0),
      new Translation2d(-TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0),
      new Translation2d(-TRACK_WIDTH_X_M / 2.0, -TRACK_WIDTH_Y_M / 2.0)
    };
  }

  /** Returns the kinematics of the drivetrain */
  public SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(getModuleTranslations());
  }

  @AutoLogOutput(key = "Drive/PP/RotationTargetOverride")
  public boolean getPPRotationTargetOverride() {
    return PProtationTargetOverride;
  }

  public void setPProtationTargetOverride(boolean override) {
    PProtationTargetOverride = override;
  }

  /** Returns the filtered pose */
  @AutoLogOutput(key = "Drive/Odometry/FilteredPose")
  public Pose2d updateFilteredPose() {
    filteredPose =
        new Pose2d(
            xFilter.calculate(getPoseEstimate().getX()),
            yFilter.calculate(getPoseEstimate().getY()),
            getPoseEstimate().getRotation());

    return filteredPose;
  }

  public Command gyroReset(){
    return new InstantCommand(() -> resetGyro());
  }
}
