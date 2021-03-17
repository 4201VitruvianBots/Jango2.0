package frc.robot.commands.autonomous.routines.simulation;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.SmartdashboardCommand;
import frc.robot.commands.autonomous.TurnInPlace;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.intake.SetIntakeStates;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;
import java.util.List;

public class OpRoutineRed extends SequentialCommandGroup {
    /* Notes:
        Approximate total Distance to Travel (By counting Pixels) = 17.55m (57.6ft)
        That means we need to go ~ 3.8 ft/s at minimum. Lets try 6 ft/s for some headroom.

        Note: Using unreasonable limits in the Sim will result in weird behavior

     */
    public OpRoutineRed(DriveTrain driveTrain, Intake intake, Indexer indexer, Turret turret, Shooter shooter, Vision vision, FieldSim fieldSim) {
//        Pose2d initPosition = new Pose2d(12.571505,6.425269, new Rotation2d(Units.degreesToRadians(0)));
//        Pose2d startTurnPosition = new Pose2d(12.571505,6.425269, new Rotation2d(Units.degreesToRadians(-22.5)));
//        Pose2d blueTrenchIntakePosA = new Pose2d(10.049316,7.166328, new  Rotation2d(Units.degreesToRadians(-55)));
//        Pose2d blueTrenchIntakePosA = new Pose2d(10.049316,7.166328, new  Rotation2d(Units.degreesToRadians(-55)));
//        Pose2d blueTrenchIntakePosB = new Pose2d(10.049316,7.166328, new  Rotation2d(Units.degreesToRadians(-22.5)));
        double startTime = Timer.getFPGATimestamp();
        Pose2d startPosition = new Pose2d(12.557047,7.275692, new Rotation2d(Units.degreesToRadians(0)));
        Pose2d blueTrenchIntakePosA = new Pose2d(10.1,7.275692, new Rotation2d(Units.degreesToRadians(0)));
        Pose2d blueTrenchIntakePosB = new Pose2d(10.1,7.275692, new Rotation2d(Units.degreesToRadians(-35)));
        Pose2d midPoint = new Pose2d(11.4,5, new Rotation2d(Units.degreesToRadians(-90)));
        Pose2d crossoverStopPos = new Pose2d(11.4,0.660829, new  Rotation2d(Units.degreesToRadians(-90)));
        Pose2d redTrenchIntakePosA = new Pose2d(11.4,0.660829, new  Rotation2d(Units.degreesToRadians(0)));
        Pose2d redTrenchIntakePosB = new Pose2d(8.607954,0.660829, new  Rotation2d(Units.degreesToRadians(0)));

        Pose2d redCenterIntakePosA = new Pose2d(8.607954,0.660829, new  Rotation2d(Units.degreesToRadians(-90)));
//        Pose2d redCenterIntakePosB = new Pose2d(9.541358,2.741749, new  Rotation2d(Units.degreesToRadians(-155)));
        Pose2d redCenterIntakePosB = new Pose2d(9.541358,2.85, new  Rotation2d(Units.degreesToRadians(-155)));


        /*  Note For some reason, a curved reverse path ends up looking like a question mark, where the robot turns outwards
            first before going to the balls. Don't know how to get around this, so using a simpler
         */
//        Trajectory startToBlueTrajectory = TrajectoryGenerator.generateTrajectory(initPosition,
//                                                                                List.of(),
//                                                                                blueTrenchIntakePos,
//                                                                                configA);

//        Transform2d transform = startTurnPosition.minus(simpleReversePath.getInitialPose());
//        Trajectory startToBlueTrajectory = simpleReversePath.transformBy(transform);

        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(8), Units.feetToMeters(40));
        configA.setReversed(true);
        var simpleReversePath = TrajectoryGenerator.generateTrajectory(startPosition,
                List.of(),
                blueTrenchIntakePosA,
                configA);

        var startToBlueTrench = TrajectoryUtils.generateRamseteCommand(driveTrain, simpleReversePath);

        TrajectoryConfig turnTrajectoryConfig = new TrajectoryConfig(Units.feetToMeters(7), Units.feetToMeters(40));
        turnTrajectoryConfig.setReversed(false);
        turnTrajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(0.75));
        turnTrajectoryConfig.setEndVelocity(Units.feetToMeters(7));

        Trajectory crossoverTrajectoryA = TrajectoryGenerator.generateTrajectory(blueTrenchIntakePosB,
                List.of(),
                midPoint,
                turnTrajectoryConfig);

        var crossoverCommandA = TrajectoryUtils.generateRamseteCommand(driveTrain, crossoverTrajectoryA);

        TrajectoryConfig driveStraghtTrajectoryConfig = new TrajectoryConfig(Units.feetToMeters(8), Units.feetToMeters(40));
        driveStraghtTrajectoryConfig.setReversed(false);
        driveStraghtTrajectoryConfig.setStartVelocity(Units.feetToMeters(7));

        Trajectory crossoverTrajectoryB = TrajectoryGenerator.generateTrajectory(midPoint,
                List.of(),
                crossoverStopPos,
                driveStraghtTrajectoryConfig);

        var crossoverCommandB = TrajectoryUtils.generateRamseteCommand(driveTrain, crossoverTrajectoryB);

        driveStraghtTrajectoryConfig.setReversed(true);
        driveStraghtTrajectoryConfig.setStartVelocity(0);
        Trajectory redTrenchIntakeTrajectory = TrajectoryGenerator.generateTrajectory(redTrenchIntakePosA,
                List.of(),
                redTrenchIntakePosB,
                driveStraghtTrajectoryConfig);

        var redTrenchIntakeCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, redTrenchIntakeTrajectory);

        turnTrajectoryConfig.setReversed(true);
        turnTrajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(0.75));
        turnTrajectoryConfig.setEndVelocity(Units.feetToMeters(6));

        Trajectory redCenterIntakeTrajectory = TrajectoryGenerator.generateTrajectory(redCenterIntakePosA,
                List.of(),
                redCenterIntakePosB,
                turnTrajectoryConfig);

        var redCenterIntakeCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, redCenterIntakeTrajectory);


        addCommands(
                new SetOdometry(driveTrain, fieldSim, startPosition),
                new SetDriveShifters(driveTrain, false),
                new ParallelDeadlineGroup(startToBlueTrench,
                        new SetIntakeStates(intake, true, 1))
                        .andThen(() -> driveTrain.setVoltageOutput(0,0)),
                new TurnInPlace(driveTrain, -35),
                new WaitCommand(0.1),
                crossoverCommandA,   //.andThen(() -> driveTrain.setVoltageOutput(0,0))
                new ParallelDeadlineGroup(crossoverCommandB,
                        new SimulationShoot(fieldSim, true)),
                new TurnInPlace(driveTrain, 0),
                redTrenchIntakeCommand,
                new ParallelDeadlineGroup(new TurnInPlace(driveTrain, -90),
                        new SimulationShoot(fieldSim, true)),
                redCenterIntakeCommand.andThen(() -> driveTrain.setVoltageOutput(0,0)),
                new ParallelDeadlineGroup(new WaitCommand(1.6),
                        new SimulationShoot(fieldSim, true))



//                new ParallelDeadlineGroup(new WaitCommand(2),
//                                          new SimulationShoot(fieldSim, true)),
//                startToRedTrench,
//                new WaitCommand(2)
//                .andThen(trenchToShootCommand)
//                .alongWith(new SetTurretRobotRelativeAngle(turret, 0))
//                .andThen(() -> driveTrain.setMotorTankDrive(0,0))
//                .andThen(new AutoUseVisionCorrection(turret, vision).withTimeout(0.75))
//                .andThen(new ParallelDeadlineGroup(new WaitCommand(2),
//                                                   new SimulationShoot(fieldSim, true)))
        );

    }
}

