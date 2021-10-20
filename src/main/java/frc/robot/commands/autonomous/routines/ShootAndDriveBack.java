package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.ResetOdometry;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.shooter.AutoRapidFireSetpoint;
import frc.robot.commands.shooter.SetAndHoldRpmSetpoint;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;

public class ShootAndDriveBack extends SequentialCommandGroup {
    public ShootAndDriveBack(DriveTrain driveTrain, Intake intake, Indexer indexer, Turret turret, Shooter shooter, Vision vision) {
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(4), Units.feetToMeters(2));
        config.setReversed(true);
        config.setEndVelocity(0);
        ArrayList<Pose2d> path = new ArrayList<>();
        path.add(new Pose2d(0, 0, new Rotation2d()));
        path.add(new Pose2d(Units.feetToMeters(- 5), 0, new Rotation2d()));
        var driveBackwards = TrajectoryUtils.generateVitruvianRamseteCommand(driveTrain, path, config);

        addCommands(
                new ResetOdometry(driveTrain),
                new SetDriveNeutralMode(driveTrain, 0),
                new SetDriveShifters(driveTrain, false),
                new SetAndHoldRpmSetpoint(shooter, vision, 3800),
                new SetTurretRobotRelativeAngle(turret, 0).withTimeout(0.5),
                new AutoUseVisionCorrection(turret, vision).withTimeout(0.5),
//                new WaitCommand(0.5),
                new ConditionalCommand(new WaitCommand(0),
                        new WaitCommand(0.5),
                        shooter :: canShoot),
                new AutoRapidFireSetpoint(shooter, indexer, intake, 1).withTimeout(3),
                driveBackwards.andThen(() -> driveTrain.setMotorTankDrive(0, 0))
        );
    }
}

