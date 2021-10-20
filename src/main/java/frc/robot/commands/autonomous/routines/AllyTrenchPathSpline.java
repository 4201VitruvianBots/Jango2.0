package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.ResetOdometry;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.commands.intake.AutoControlledIntake;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.shooter.AutoRapidFireSetpoint;
import frc.robot.commands.shooter.SetAndHoldRpmSetpoint;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

import javax.xml.crypto.dsig.Transform;
import java.util.ArrayList;

public class AllyTrenchPathSpline extends SequentialCommandGroup {
    public AllyTrenchPathSpline(DriveTrain driveTrain, Intake intake, Indexer indexer, Turret turret, Shooter shooter, Vision vision) {
        Pose2d startPosition = new Pose2d(Units.inchesToMeters(145), 7.5, new Rotation2d(Units.degreesToRadians(180)));
        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(10));
        configA.setReversed(true);
        configA.setEndVelocity(0);
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        //var startToTrenchPath = TrajectoryUtils.readCsvTrajectory("init1Ally2");
        ArrayList<Pose2d> startToTrenchPath = new ArrayList();
        startToTrenchPath.add(startPosition);
        startToTrenchPath.add(new Pose2d(Units.inchesToMeters(276), 7.5, new Rotation2d(Units.degreesToRadians(180))));
        var startToTrenchCommand = TrajectoryUtils.generateVitruvianRamseteCommand(driveTrain, startToTrenchPath, configA);

        Pose2d midPoint = new Pose2d(Units.feetToMeters(-13), 0, new Rotation2d(0));
        var configB = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(4));
        configB.setReversed(false);
        configB.setEndVelocity(0);
        configB.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configB.getMaxVelocity()));
        configB.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        configB.addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(1.5)));
        //var trenchToShootPath = TrajectoryUtils.readCsvTrajectory("ally2Ally3");
        ArrayList<Pose2d> trenchToShootPath = new ArrayList();
        trenchToShootPath.add(new Pose2d(Units.inchesToMeters(276), 7.5, new Rotation2d(Units.degreesToRadians(180))));
        trenchToShootPath.add(new Pose2d(Units.inchesToMeters(145), 5.9, new Rotation2d(Units.degreesToRadians(180))));
        var trenchToShootCommand = TrajectoryUtils.generateVitruvianRamseteCommand(driveTrain, trenchToShootPath, configB);

        if(RobotBase.isReal())
            addCommands(
                    new SetOdometry(driveTrain, startPosition),
                    new SetDriveNeutralMode(driveTrain,0),
                    new SetDriveShifters(driveTrain, false),
                    new SetAndHoldRpmSetpoint(shooter, vision, 3800),
                    new SetTurretRobotRelativeAngle(turret, -25).withTimeout(0.5),
                    new AutoUseVisionCorrection(turret, vision).withTimeout(0.5),
                    new ConditionalCommand(new WaitCommand(0),
                                           new WaitCommand(0.5),
                                           shooter::canShoot),
                    new AutoRapidFireSetpoint(shooter, indexer, intake,1).withTimeout(1),
                    new SetIntakePiston(intake, true),
                    new SetDriveShifters(driveTrain, false),
                    new ParallelDeadlineGroup(
                            startToTrenchCommand,
                            new AutoControlledIntake(intake, indexer)
                    ),
                    new AutoControlledIntake(intake, indexer).withTimeout(0.5),
                    new SetIntakePiston(intake, false),
                    new ParallelDeadlineGroup(
                            trenchToShootCommand,
                            new SetTurretRobotRelativeAngle(turret, 0),
                            new SetAndHoldRpmSetpoint(shooter, vision, 3800)
                    ).andThen(()->driveTrain.setMotorTankDrive(0,0)),
                    new AutoUseVisionCorrection(turret, vision).withTimeout(0.75),
                    new ConditionalCommand(new WaitCommand(0),
                                           new WaitCommand(0.5),
                                           shooter::canShoot),
                    new ConditionalCommand(new AutoRapidFireSetpoint(shooter, indexer, intake,6),
                                           new WaitCommand(0),
                                           vision::hasTarget)
            );
        else
            addCommands(
                    new SetOdometry(driveTrain, startPosition),
                    new SetTurretRobotRelativeAngle(turret, -25).withTimeout(0.5),
                    new AutoUseVisionCorrection(turret, vision).withTimeout(0.5),
                    startToTrenchCommand,
                    new WaitCommand(2)
                    .andThen(trenchToShootCommand)
                    .alongWith(new SetTurretRobotRelativeAngle(turret, 0))
                    .andThen(new AutoUseVisionCorrection(turret, vision).withTimeout(0.75))
            );
    }
}

