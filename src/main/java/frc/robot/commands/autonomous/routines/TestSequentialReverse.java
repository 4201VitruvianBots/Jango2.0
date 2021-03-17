package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.TestPath;
import frc.robot.subsystems.DriveTrain;

import java.util.ArrayList;

public class TestSequentialReverse extends SequentialCommandGroup {
    public TestSequentialReverse(DriveTrain driveTrain) {
        ArrayList<Pose2d> pathA = new ArrayList<>();
        pathA.add(new Pose2d(0, 0, new Rotation2d()));
        pathA.add(new Pose2d(5, 0, new Rotation2d()));
        ArrayList<Pose2d> pathB = new ArrayList<>();
        pathB.add(new Pose2d(5, 0, new Rotation2d()));
        pathB.add(new Pose2d(10, - 5, new Rotation2d()));
        pathB.add(new Pose2d(5, - 10, new Rotation2d(Units.degreesToRadians(- 180))));

        addCommands(new TestPath(driveTrain, pathA, true),
                new TestPath(driveTrain, pathB, true));
    }
}
