package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * 0.5;
  private double MaxAngularRate = 1.5 * Math.PI; 

  public double getVoltage() {

    return  

    drivetrain.getModule(0).getDriveMotor().get();

  }

    public double getVelocity() {

    return  

    drivetrain.getModule(0).getDriveMotor().getVelocity().getValueAsDouble();

  }

  private final CommandXboxController joystick = new CommandXboxController(0);
  
  private final Swerve drivetrain = TunerConstants.DriveTrain; 

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.Velocity); 

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate))); // Drive counterclockwise with negative X (left)

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Re-Gyro the robot
    joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {

      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));

    }

    drivetrain.registerTelemetry(logger::telemeterize);

  }

  public RobotContainer() {

    configureBindings();

  }

  public Command getAutonomousCommand() {

    return Commands.print("No autonomous command configured");

  }

}
