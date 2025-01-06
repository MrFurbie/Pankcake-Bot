package frc.robot.subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants;
import frc.robot.DriveConstants;

public class Swerve extends SwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms

    private final Swerve drivetrain = DriveConstants.DriveTrain;

    private Notifier m_simNotifier = null;

    private double m_lastSimTime;

    private final PIDController turnController =
        new PIDController(
            Constants.SwerveConstants.turnControllerP,
            Constants.SwerveConstants.turnControllerI,
            Constants.SwerveConstants.turnControllerD);

    private final double MaxSpeed = DriveConstants.kSpeedAt12VoltsMps * 0.7;
    private final double MaxAngularRate = 1.55 * Math.PI;

    public double getMaximumVelocity() {
        return DriveConstants.kSpeedAt12VoltsMps * 0.7;
    }

    public double getMaximumAngularVelocity() {
        return 1.55 * Math.PI;
    }

    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.07) // Add a 7% deadband
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

    // New getHeading method
    private Rotation2d getHeading() {
        return Rotation2d.fromDegrees(super.getYaw()); // Assuming `getYaw()` is available in the parent class
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command driveCommand(
        DoubleSupplier translationX,
        DoubleSupplier translationY,
        DoubleSupplier angularRotationX,
        BooleanSupplier doAim,
        PhotonCamera camera) {

        return run(() -> {
            PhotonPipelineResult result = camera.getLatestResult();
            double yaw = 0.0;

            if (result.hasTargets() && doAim.getAsBoolean()) {
                List<PhotonTrackedTarget> targets = result.getTargets();
                for (PhotonTrackedTarget target : targets) {
                    if (target.getFiducialId() == 7 || target.getFiducialId() == 4) {
                        yaw = target.getYaw();
                        break;
                    }
                }

                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    MathUtil.applyDeadband(translationX.getAsDouble() * MaxSpeed, Constants.SwerveConstants.swerveDeadband),
                    MathUtil.applyDeadband(translationY.getAsDouble() * MaxSpeed, Constants.SwerveConstants.swerveDeadband),
                    turnController.calculate(yaw, 0),
                    getHeading());

                this.setControl(SwerveRequest.fromChassisSpeeds(speeds));
            } else {
                ChassisSpeeds speeds = new ChassisSpeeds(
                    MathUtil.applyDeadband(translationX.getAsDouble() * MaxSpeed, Constants.SwerveConstants.swerveDeadband),
                    MathUtil.applyDeadband(translationY.getAsDouble() * MaxSpeed, Constants.SwerveConstants.swerveDeadband),
                    MathUtil.applyDeadband(angularRotationX.getAsDouble() * MaxAngularRate, Constants.SwerveConstants.swerveDeadband));

                this.setControl(SwerveRequest.fromChassisSpeeds(speeds));
            }
        });
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(alliance -> {
                setOperatorPerspectiveForward(alliance == Alliance.Red ? RedAlliancePerspectiveRotation : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
}
