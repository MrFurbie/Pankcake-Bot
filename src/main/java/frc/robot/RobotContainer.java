package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.Swerve;

public class RobotContainer {
    
    private double MaxSpeed = DriveConstants.kSpeedAt12Volts * 0.5; // 0.1 is about 0.5 mps, 0.9 / 90% is max, go no higher
                                                                    // Value should be tuned with new 2025 code

    private double MaxAngularRate = 1.25 * Math.PI; // Controls how fast the robot quick turns

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.07) // Add a 7% deadband
    .withDriveRequestType(DriveRequestType.Velocity); 

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final Swerve drivetrain = DriveConstants.createDrivetrain();

    private final CommandXboxController joystick = new CommandXboxController(0);

     PIDController xPID = new PIDController(3, 0, 0);
     PIDController yPID = new PIDController(3, 0, 0);
     PIDController rPID = new PIDController(3, 0, 0);

       public boolean isRed() {

    if (DriverStation.getAlliance().isPresent()) {

      return DriverStation.getAlliance().get() ==  Alliance.Red;

    }

    return false;

    }

    private final SwerveRequest.ApplyChassisSpeeds chassisSpeedRequest = new SwerveRequest.ApplyChassisSpeeds();

    public RobotContainer() {
        
        configureAxisActions();

        configureBindings();


    }

    private void configureAxisActions() {

    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate))); // Drive counterclockwise with negative X (left)

    }

    private void configureBindings() {

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() -> 
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }


     public void runChassisSpeeds(ChassisSpeeds speeds){ 

    drivetrain.setControl(chassisSpeedRequest.withSpeeds(speeds));

  }

  public Command getAutonomousCommand() {

    return null;

  }

}