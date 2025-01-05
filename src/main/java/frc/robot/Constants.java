package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;


public class Constants {
    
    public static final class VisionConstants {

    public static final String visionName = "USB_GS_Camera";
    public static final String driverName = "HD_Web_Camera";

    public static final Pair<Integer, Translation2d> blueSpeaker =
        new Pair<Integer, Translation2d>(7, new Translation2d(0.00, 5.55));
    public static final Pair<Integer, Translation2d> redSpeaker =
        new Pair<Integer, Translation2d>(4, new Translation2d(15.64, 5.55));

    public static final Transform3d robotToCam =
        new Transform3d(
            Units.inchesToMeters(-11.0),
            Units.inchesToMeters(7.0),
            Units.inchesToMeters(16.5),
            new Rotation3d(0.0, Units.degreesToRadians(105.0), Units.degreesToRadians(180.0)));

    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

        public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();

  }

  public static final class SwerveConstants {

    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final double swerveDeadband = 0.1;

    public static final double turnControllerP = 0.1;
    public static final double turnControllerI = 0.0;
    public static final double turnControllerD = 0.0;

    public static final double maximumSpeed = Units.feetToMeters(19.0);
    
  }



}
