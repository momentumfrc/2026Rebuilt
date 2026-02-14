package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.IndexerSubsystem;
import frc.robot.subsystem.KickerSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.NTHelpers;
import frc.robot.util.OdometryTargetingHelper;
import java.util.function.Supplier;

public class ShootCommand extends Command {
    private enum TurretAlignMode {
        LIMELIGHT_FINE_ALIGN,
        ODOMETRY_ONLY
    };

    private final KickerSubsystem kicker;
    private final IndexerSubsystem indexer;
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final PositioningSubsystem positioning;

    private final Supplier<MoInput> inputSupplier;

    private final SendableChooser<TurretAlignMode> turretAlignMode = NTHelpers.enumToChooser(TurretAlignMode.class);

    private final MutAngle turretSetpointAngle = Units.Radians.mutable(0);

    public ShootCommand(
            KickerSubsystem kicker,
            IndexerSubsystem indexer,
            TurretSubsystem turret,
            ShooterSubsystem shooter,
            PositioningSubsystem positioning,
            Supplier<MoInput> inputSupplier) {

        this.kicker = kicker;
        this.indexer = indexer;
        this.turret = turret;
        this.shooter = shooter;
        this.positioning = positioning;

        this.inputSupplier = inputSupplier;

        addRequirements(this.kicker, this.indexer, this.turret, this.shooter);
    }

    private void alignTurretOdometry() {
        Pose2d robotPose = positioning.getRobotPose();
        Translation2d toTarget = OdometryTargetingHelper.getTranslationToTarget(
                robotPose.getTranslation(), DriverStation.getAlliance().orElse(DriverStation.Alliance.Red));
        turretSetpointAngle.mut_replace(toTarget.getAngle().getRadians(), Units.Radians);
        turretSetpointAngle.mut_minus(robotPose.getRotation().getRadians(), Units.Radians);
        turret.alignAbsolute(turretSetpointAngle);
    }

    private void alignTurretRelative() {
        if (turret.relativeTargetIsVisible()) {
            turret.alignRelative();
        } else {
            alignTurretOdometry();
        }
    }

    private boolean turretIsAligned(TurretAlignMode mode) {
        return switch (mode) {
            case LIMELIGHT_FINE_ALIGN -> turret.relativeTargetIsAligned();
            case ODOMETRY_ONLY -> turret.absoluteTargetIsAligned();
        };
    }

    private void alignTurret(TurretAlignMode mode) {
        switch (mode) {
            case LIMELIGHT_FINE_ALIGN:
                alignTurretRelative();
                break;
            case ODOMETRY_ONLY:
                alignTurretOdometry();
                break;
        }
    }

    private void spinupKicker() {
        kicker.
    }

    public void doShoot(boolean run) {
        if (run) {
            kicker.run();
            indexer.run();
        } else {
            kicker.stop();
            indexer.stop();
        }
    }

    @Override
    public void execute() {
        doShoot(input.getShootRequest());
    }
}
