package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.actions.CancelableAction;
import org.firstinspires.ftc.teamcode.actions.SmartEmptyMag;
import org.firstinspires.ftc.teamcode.actions.WaitUntilBB;
import org.firstinspires.ftc.teamcode.actions.WaitUntilEmptyBB;
import org.firstinspires.ftc.teamcode.actions.WaitUntilThird;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Gate;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.util.hardware.Breakbeam;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaLEDIndicator;
import org.firstinspires.ftc.teamcode.util.helpers.AutoShoot;
import org.firstinspires.ftc.teamcode.util.helpers.BallCounter;
import org.firstinspires.ftc.teamcode.util.helpers.MatchStorage;
import org.firstinspires.ftc.teamcode.util.models.ShotSolution;
import org.firstinspires.ftc.teamcode.util.wrappers.Motor;
import org.firstinspires.ftc.teamcode.util.wrappers.StepperServo;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {

    public HardwareMap hardwareMap;

    public Motor backLeft;
    public Motor backRight;
    public Motor frontLeft;
    public Motor frontRight;

    public Intake intake;
    public Transfer transfer;
    public Flywheel flywheel;
    public Gate gate;
    public Hood hood;
    public Turret turret;

    public GoBildaPinpointDriver pinpoint;
    public Limelight3A limelight3A;
    public GoBildaLEDIndicator blinky;
    public BallCounter counter;

    public PIDController headingPID;
    public final double hP = 0.01, hI = 0.00, hD = 0.0001, hF = 0.05;
    public final double hE = 2.0;

    public boolean auton;
    public boolean intaking = false;
    public boolean shooting = false;
    public boolean gateState = false;
    public boolean isFull = false;

    public Pose2D pose;
    public Pose2D virtualTarget;
    public double turretManualOffset = 0;

    ElapsedTime timer = new ElapsedTime();
    public static double time;

    public Robot(HardwareMap map, boolean auton) {
        this.hardwareMap = map;
        this.auton = auton;

        if (!auton) {
            backLeft = new Motor(2, "leftBack", map, true);
            backRight = new Motor(1, "rightBack", map, false);
            frontLeft = new Motor(1, "leftFront", map, true);
            frontRight = new Motor(2, "rightFront", map, false);

            headingPID = new PIDController(hP, hI, hD);
        }

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(-8.5, -110, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.start();

        blinky = new GoBildaLEDIndicator(0, "blinky", map);

        Breakbeam bb1 = new Breakbeam(0, "bb1", map);
        Breakbeam bb2 = new Breakbeam(2, "bb2", map);
        Breakbeam bb3 = new Breakbeam(4, "bb3", map);
        counter = new BallCounter(bb1, bb2, bb3);

        intake = new Intake(new Motor(3, "intake", map, false));
        transfer = new Transfer(new Motor(2, "transfer", map, false));

        Motor fw1 = new Motor(3, "flywheel1", map, true);
        Motor fw2 = new Motor(0, "flywheel2", map, false);
        flywheel = new Flywheel(fw1, fw2, null);

        gate = new Gate(new StepperServo(0, "gate", map));
        hood = new Hood(new StepperServo(1, "hood", map));

        Motor turretMotor = new Motor(4, "turret", map, true);
        AnalogInput turretEncoder = map.get(AnalogInput.class, "absTurretEncoder");
        turret = new Turret(turretMotor, turretEncoder, null);
    }

    public void update() {
        turret.update();
        flywheel.update();
        blinky.update();
        pinpoint.update();
        counter.refresh();
    }

    public void updateAim() {
        pose = pinpoint.getPosition();
        double velX = pinpoint.getVelX(DistanceUnit.INCH);
        double velY = pinpoint.getVelY(DistanceUnit.INCH);

        virtualTarget = AutoShoot.calculateNormalizedVirtualTarget(
                velX,
                velY,
                pinpoint.getHeadingVelocity(AngleUnit.RADIANS),
                pose,
                MatchStorage.getAllianceColor()
        );

        ShotSolution sol = AutoShoot.calculateShot(pose, virtualTarget, pinpoint.getHeadingVelocity(AngleUnit.RADIANS));

        flywheel.setRPM(sol.flywheelSpeed);
        turret.runToAngle(sol.turretAngle + turretManualOffset);
        hood.runToPosition(sol.hoodAngle);
    }

    public void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = (y - x + rx) * -1;
        double powerBackRight = (y + x - rx) * -1;

        if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 || Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
            double max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
            max = Math.max(Math.abs(powerFrontRight), max);
            max = Math.max(Math.abs(powerBackRight), max);

            powerFrontLeft /= max;
            powerBackLeft /= max;
            powerFrontRight /= max;
            powerBackRight /= max;
        }
        frontLeft.setPower((float) powerFrontLeft);
        frontRight.setPower((float) powerFrontRight);
        backLeft.setPower(-(float) powerBackLeft);
        backRight.setPower(-(float) powerBackRight);
    }

    public void setDrivePowerConstHeading(double x, double y, double heading, double curHeading) {
        double rx = -headingPID.calculate(curHeading, heading);
        if (Math.abs(curHeading - heading) > hE) {
            rx += rx / Math.abs(rx) * hF;
        }
        rx = Range.clip(rx, -1, 1);

        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = (y - x + rx) * -1;
        double powerBackRight = (y + x - rx) * -1;

        if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 || Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
            double max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
            max = Math.max(Math.abs(powerFrontRight), max);
            max = Math.max(Math.abs(powerBackRight), max);

            powerFrontLeft /= max;
            powerBackLeft /= max;
            powerFrontRight /= max;
            powerBackRight /= max;
        }
        frontLeft.setPower((float) powerFrontLeft);
        frontRight.setPower((float) powerFrontRight);
        backLeft.setPower(-(float) powerBackLeft);
        backRight.setPower(-(float) powerBackRight);
    }

    public Action startIntakeAndTransfer() {
        return new SequentialAction(
                new InstantAction(() -> {
                    intaking = true;
                    gateState = true;
                    gate.close();
                    intake.start();
                }),
                new WaitUntilBB(this),
                new InstantAction(intake::eject),
                new SleepAction(0.08),
                new InstantAction(intake::stall),
                new SleepAction(0.01),
                new InstantAction(() -> intaking = false)
        );
    }

    public Action startIntakeAndTransfer(Gamepad gamepad) {
        return new SequentialAction(
                new InstantAction(() -> {
                    intaking = true;
                    gateState = true;
                    gate.close();
                    intake.start();
                    blinky.set(GoBildaLEDIndicator.Colors.BLUE, GoBildaLEDIndicator.Animation.SLOW_BLINK);
                })
        );
    }

    public Action stopIntakeAndTransfer() {
        return new SequentialAction(
                new InstantAction(intake::stall),
                new SleepAction(0.01),
                new InstantAction(() -> intaking = false)
        );
    }

    public Action hoodOffsetAction() {
        return new SequentialAction(
                new WaitUntilThird(this),
                new InstantAction(() -> {
                    hood.offset = -0.12f;
                })
        );
    }

    public Action shootingAction() {
        boolean far;

        shooting = true;
        if (MatchStorage.getAllianceColor() == AllianceColor.RED) {
            far = Math.hypot((pose.getX(DistanceUnit.INCH) + 72), (72 - pose.getY(DistanceUnit.INCH))) >= 100;
        } else {
            far = Math.hypot((pose.getX(DistanceUnit.INCH) + 72), (pose.getY(DistanceUnit.INCH) + 72)) >= 100;
        }

        CancelableAction hoodOffset = new CancelableAction(hoodOffsetAction());
        return new SequentialAction(
                new InstantAction(() -> {
                    gate.open();
                    timer.reset();
                    if (far) {
                        intake.setPower(0.9);
                    } else {
                        intake.setPower(1);
                    }
                }),
                new SleepAction(0.06),
                far ? new ParallelAction(
                        new SequentialAction(
                                new SmartEmptyMag(this),
                                new InstantAction(hoodOffset::cancel)),
                        hoodOffset
                ) : new SmartEmptyMag(this),
                new InstantAction(() -> { hood.offset = 0; }),
                new InstantAction(intake::start),
                new SleepAction(0.5),
                new InstantAction(() -> {
                    gateState = false;
                    gate.close();
                    time = timer.time();
                    shooting = false;
                    isFull = false;
                })
        );
    }

    public Action autonShootFar() {
        shooting = true;
        CancelableAction hoodOffset = new CancelableAction(hoodOffsetAction());
        return new SequentialAction(
                new InstantAction(() -> {
                    gate.open();
                    timer.reset();
                    intake.setPower(0.75);
                }),
                new SleepAction(0.06),
                new ParallelAction(
                        new SequentialAction(
                                new WaitUntilEmptyBB(this),
                                new InstantAction(hoodOffset::cancel)),
                        hoodOffset
                ),
                new InstantAction(() -> {
                    hood.offset = 0;
                }),
                new InstantAction(intake::start),
                new SleepAction(0.05),
                new InstantAction(() -> {
                    gateState = false;
                    gate.close();
                    time = timer.time();
                    shooting = false;
                    blinky.setColor(GoBildaLEDIndicator.Colors.JOOS_ORANGE);
                })
        );
    }

    public Action autonShootClose() {
        shooting = true;
        return new SequentialAction(
                new InstantAction(() -> {
                    shooting = true;
                    gate.open();
                    timer.reset();
                    intake.setPower(1);
                }),
                new SleepAction(0.06),
                new WaitUntilEmptyBB(this),
                new InstantAction(intake::start),
                new SleepAction(0.2),
                new InstantAction(() -> {
                    gateState = false;
                    gate.close();
                    time = timer.time();
                    shooting = false;
                })
        );
    }
}