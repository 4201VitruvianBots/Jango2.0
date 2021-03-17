package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.ResetOdometry;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;

public class DriveBackwards extends SequentialCommandGroup {
    public DriveBackwards(DriveTrain driveTrain) {
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(4), Units.feetToMeters(2));
        config.setReversed(false);
        ArrayList<Pose2d> path = new ArrayList<>();
        path.add(new Pose2d(0, 0, new Rotation2d()));
        path.add(new Pose2d(Units.feetToMeters(4), 0, new Rotation2d()));
        var driveBackwards = TrajectoryUtils.generateRamseteCommand(driveTrain, path, config);

        addCommands(new ResetOdometry(driveTrain),
                driveBackwards);
    }
}
