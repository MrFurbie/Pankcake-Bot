package frc.robot.subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.io.File;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.DriveConstants;
import frc.robot.RobotContainer;

public class Swerve extends SwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms

    private final Swerve drivetrain = DriveConstants.DriveTrain;

    private Notifier m_simNotifier = null;

    private double m_lastSimTime;

    private double MaxSpeed = DriveConstants.kSpeedAt12VoltsMps * 0.7;

    private double MaxAngularRate = 1.55 * Math.PI;

    private final PIDController turnController =
    new PIDController(
        Constants.SwerveConstants.turnControllerP,
        Constants.SwerveConstants.turnControllerI,
        Constants.SwerveConstants.turnControllerD);

    public double getMaximumVelocity() {

        return DriveConstants.kSpeedAt12VoltsMps * 0.7;

    }

    public double getMaximumAngularVelocity() {

        return 1.55 * Math.PI; 


    }

    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.07) // Add a 7% deadband
    .withDriveRequestType(DriveRequestType.Velocity); 

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);

    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {

        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        if (Utils.isSimulation()) {

            startSimThread();

        }

    }
    
    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {

        super(driveTrainConstants, modules);

        if (Utils.isSimulation()) {

            startSimThread();

        }

    }

    public Command driveCommand(

      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier angularRotationX,

      BooleanSupplier doAim,

      PhotonCamera camera) {

    return run(
        () -> {
          PhotonPipelineResult result = camera.getLatestResult();
          double yaw = 0.0;
          if (result.hasTargets() && doAim.getAsBoolean()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            for (int i = 0; i < targets.size(); i++) {
              if (targets.get(i).getFiducialId() == 7 || targets.get(i).getFiducialId() == 4) {
                yaw = targets.get(i).getYaw();
              }
            }
            drive(
                drivetrain.swerveController.getRawTargetSpeeds(
                    MathUtil.applyDeadband(
                        translationX.getAsDouble() * drivetrain.getMaximumVelocity(),
                        Constants.SwerveConstants.swerveDeadband),
                    MathUtil.applyDeadband(
                        translationY.getAsDouble() * drivetrain.getMaximumVelocity(),
                        Constants.SwerveConstants.swerveDeadband),
                    -turnController.calculate(yaw, 0)));
          } else {

            // drivetrain.drive(
            //     new Translation2d(
            //         MathUtil.applyDeadband(
            //             translationX.getAsDouble() * drivetrain.getMaximumVelocity(),
            //             Constants.SwerveConstants.swerveDeadband),
            //         MathUtil.applyDeadband(
            //             translationY.getAsDouble() * drivetrain.getMaximumVelocity(),
            //             Constants.SwerveConstants.swerveDeadband)),
            //     MathUtil.applyDeadband(
            //         angularRotationX.getAsDouble() * drivetrain.getMaximumAngularVelocity(),
            //         Constants.SwerveConstants.swerveDeadband),
            //     true,
            //     false);

            drivetrain.applyRequest(() -> drive.withVelocityX(translationY.getAsDouble() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(translationX.getAsDouble() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(angularRotationX.getAsDouble() * MaxAngularRate)); // Drive counterclockwise with negative X (left)

          }

        });
  }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {

        return run(() -> this.setControl(requestSupplier.get()));

    }

    private void startSimThread() {

        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {

            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());

        });

        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    
    public void periodic() {

        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/

        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {

            DriverStation.getAlliance().ifPresent((allianceColor) -> {

                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);

                hasAppliedOperatorPerspective = true;

            });

        }

    }
    
}
