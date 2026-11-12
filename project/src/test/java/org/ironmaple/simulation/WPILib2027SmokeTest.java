package org.ironmaple.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.utils.LegacyFieldMirroringUtils2024;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;

/** Headless smoke tests that guard the WPILib 2027 port: the simulation must still behave physically. */
class WPILib2027SmokeTest {
    private SwerveDriveSimulation drive;
    private SimulatedMotorController.GenericMotorController[] driveControllers;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize());
        SimulatedArena.overrideInstance(new Arena2026Rebuilt());
        drive = new SwerveDriveSimulation(DriveTrainSimulationConfig.Default(), new Pose2d(4, 4, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(drive);
        driveControllers = new SimulatedMotorController.GenericMotorController[4];
        for (int i = 0; i < 4; i++) {
            driveControllers[i] = drive.getModules()[i].useDriveMotorController(
                    new SimulatedMotorController.GenericMotorController(DCMotor.getKrakenX60(1)));
            drive.getModules()[i].useSteerMotorController(
                    new SimulatedMotorController.GenericMotorController(DCMotor.getKrakenX60(1)));
        }
    }

    private void driveForOneSecond(double volts) {
        for (int tick = 0; tick < 50; tick++) {
            for (var c : driveControllers) c.requestVoltage(Volts.of(volts));
            SimulatedArena.getInstance().simulationPeriodic();
        }
    }

    @Test
    void drivesInAStraightLineWhenModulesAreAligned() {
        driveForOneSecond(8);
        Pose2d pose = drive.getSimulatedDriveTrainPose();
        assertTrue(Math.abs(pose.getX() - 4.0) > 0.3, "robot should have travelled along X, was " + pose);
        assertEquals(4.0, pose.getY(), 0.1);
        assertEquals(0.0, pose.getRotation().getRadians(), 0.1);
    }

    @Test
    void fieldRelativeAndRobotRelativeSpeedsAreRelatedByTheRobotHeading() {
        drive.setSimulationWorldPose(new Pose2d(4, 4, Rotation2d.fromDegrees(90)));
        driveForOneSecond(8);
        ChassisVelocities field = drive.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        ChassisVelocities robot = drive.getDriveTrainSimulatedChassisSpeedsRobotRelative();
        double heading = drive.getSimulatedDriveTrainPose().getRotation().getRadians();
        assertTrue(Math.hypot(field.vx, field.vy) > 0.5, "robot should be moving: " + field);
        assertEquals(field.vx * Math.cos(heading) + field.vy * Math.sin(heading), robot.vx, 1e-6);
        assertEquals(-field.vx * Math.sin(heading) + field.vy * Math.cos(heading), robot.vy, 1e-6);
    }

    @Test
    void allianceFacingDefaultsToRedWhenNoDriverStation() {
        assertEquals(
                Math.PI,
                LegacyFieldMirroringUtils2024.getCurrentAllianceDriverStationFacing()
                        .getRadians(),
                1e-9);
    }
}
