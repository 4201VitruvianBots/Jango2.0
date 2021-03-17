package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.SetTankDrive;
import frc.robot.commands.indexer.FeedAll;
import frc.robot.commands.turret.ShootOnTheMove;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;

import java.util.function.DoubleSupplier;

public class SOTMWhileMovingVariable extends ParallelCommandGroup {
    public SOTMWhileMovingVariable(DriveTrain driveTrain, ShootOnTheMove shootOnTheMove, Indexer indexer, DoubleSupplier leftPower, DoubleSupplier rightPower) {
        addCommands(
                new SOTMForTime(shootOnTheMove, 10),
                new SetTankDrive(driveTrain, leftPower, rightPower),
                new SequentialCommandGroup(new WaitCommand(5), new FeedAll(indexer))
        );
    }
}
