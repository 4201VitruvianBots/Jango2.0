package frc.robot.commands.autonomous.routines.simulation;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.commands.intake.AutoControlledIntake;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.shooter.AutoRapidFireSetpoint;
import frc.robot.commands.shooter.SetAndHoldRpmSetpoint;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;

public class OpRoutineBlue extends SequentialCommandGroup {
    public OpRoutineBlue(DriveTrain driveTrain, Intake intake, Indexer indexer, Turret turret, Shooter shooter, Vision vision, FieldSim fieldSim) {
        Pose2d startPosition = new Pose2d(3.449221,1.838556, new Rotation2d(Units.degreesToRadians(180)));
        Pose2d redTrenchIntakePos = new Pose2d(5.836136,0.905403, new  Rotation2d(Units.degreesToRadians(145)));
        Pose2d crossoverStopPos = new Pose2d(4.786788,7.268511, new  Rotation2d(Units.degreesToRadians(90)));
        Pose2d blueTrenchIntakePos = new Pose2d(7.312648,7.268511, new  Rotation2d(Units.degreesToRadians(180)));
        Pose2d blueCenterIntakePos = new Pose2d(6.566841,5.467240, new  Rotation2d(Units.degreesToRadians(155)));

        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(20), Units.feetToMeters(20));
        configA.setReversed(true);
//        configA.setEndVelocity(0);
//        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
//        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        //var startToTrenchPath = TrajectoryUtils.readCsvTrajectory("init1Ally2");
        ArrayList<Pose2d> startToTrenchPath = new ArrayList();
        startToTrenchPath.add(startPosition);
        startToTrenchPath.add(redTrenchIntakePos);
        var startToRedTrench = TrajectoryUtils.generateVitruvianRamseteCommand(driveTrain, startToTrenchPath, configA);

        var configB = new TrajectoryConfig(Units.feetToMeters(20), Units.feetToMeters(20));
        configB.setReversed(false);
//        configB.setEndVelocity(0);
//        configB.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configB.getMaxVelocity()));
//        configB.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
//        configB.addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(2)));
        //var trenchToShootPath = TrajectoryUtils.readCsvTrajectory("ally2Ally3");
        ArrayList<Pose2d> trenchToShootPath = new ArrayList();
        startToTrenchPath.add(redTrenchIntakePos);
//        Pose2d crossoverStopPos = new Pose2d(4.786788,7.268511, new  Rotation2d(Units.degreesToRadians(90)));
        var trenchToShootCommand = TrajectoryUtils.generateVitruvianRamseteCommand(driveTrain, trenchToShootPath, configB);


        addCommands(
                new SetOdometry(driveTrain, startPosition),
//                    new SetTurretRobotRelativeAngle(turret, -25).withTimeout(0.5),
                new ParallelDeadlineGroup(new WaitCommand(2),
                                          new SimulationShoot(fieldSim, true)),
                startToRedTrench,
                new WaitCommand(2)
                .andThen(trenchToShootCommand)
                .alongWith(new SetTurretRobotRelativeAngle(turret, 0))
                .andThen(() -> driveTrain.setMotorTankDrive(0,0))
                .andThen(new AutoUseVisionCorrection(turret, vision).withTimeout(0.75))
                .andThen(new ParallelDeadlineGroup(new WaitCommand(2),
                                                   new SimulationShoot(fieldSim, true))));

    }
}

