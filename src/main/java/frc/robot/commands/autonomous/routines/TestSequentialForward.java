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

public class TestSequentialForward extends SequentialCommandGroup {

    public TestSequentialForward(DriveTrain driveTrain) {
        var startConfig = new TrajectoryConfig(Units.feetToMeters(4), Units.feetToMeters(2));
        startConfig.setEndVelocity(startConfig.getMaxVelocity());
        ArrayList<Pose2d> pathA = new ArrayList<>();
        pathA.add(new Pose2d(0, 0, new Rotation2d()));
        pathA.add(new Pose2d(Units.feetToMeters(2), Units.feetToMeters(0), new Rotation2d()));

        var secondPathConfig = new TrajectoryConfig(Units.feetToMeters(4), Units.feetToMeters(2));
        ArrayList<Pose2d> pathB = new ArrayList<>();
        pathB.add(new Pose2d(0, 0, new Rotation2d()));
        pathB.add(new Pose2d(Units.feetToMeters(5), Units.feetToMeters(- 5), new Rotation2d(Units.degreesToRadians(- 90))));
        pathB.add(new Pose2d(Units.feetToMeters(0), Units.feetToMeters(- 10), new Rotation2d(Units.degreesToRadians(- 180))));

        var trajectoryCommandA = TrajectoryUtils.generateRamseteCommand(driveTrain, pathA, startConfig);
        var trajectoryCommandB = TrajectoryUtils.generateRamseteCommand(driveTrain, pathB, secondPathConfig);
        addCommands(new ResetOdometry(driveTrain),
                trajectoryCommandA,
                new ResetOdometry(driveTrain),
                trajectoryCommandB);
    }
}
