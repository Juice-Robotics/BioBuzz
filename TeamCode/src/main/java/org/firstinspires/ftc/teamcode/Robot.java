package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter;

import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.util.breakbeams.breakbeams;
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
    public Shooter shooter;


    public GoBildaPinpointDriver pinpoint;
    public Limelight3A limelight3A;


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
        limelight3A.getLatestResult().getFiducialResults();
        LLResultTypes.FiducialResult a;
        // a.getTargetPoseRobotSpace();


        shooter = new Shooter(new Motor(2, "shooter1", map, false),
                new Motor(2, "shooter2", map, false),
                new Motor(2, "shooter1", map, false) ,
                new StepperServo(1, "gate", map, "none"),
                new StepperServo(1, "hood", map, "none"));
        intake = new Intake(new Motor(3, "intake", map, false));
        transfer = new Transfer(new Motor(2, "transfer", map, false));

        Motor fw1 = new Motor(3, "flywheel1", map, true);
        Motor fw2 = new Motor(0, "flywheel2", map, false);

    }

    public void update() {
        pinpoint.update();
    }

    public void updateAim() {
        pose = pinpoint.getPosition();
        double velX = pinpoint.getVelX(DistanceUnit.INCH);
        double velY = pinpoint.getVelY(DistanceUnit.INCH);


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
                    intake.start();
                }),
                new SleepAction(0.08),
                new SleepAction(0.01),
                new InstantAction(() -> intaking = false)
        );
    }

    public Action startIntakeAndTransfer(Gamepad gamepad) {
        return new SequentialAction(
                new InstantAction(() -> {
                    intaking = true;
                    gateState = true;
                    intake.start();
                })
        );
    }

    public Action stopIntakeAndTransfer() {
        return new SequentialAction(
                new SleepAction(0.01),
                new InstantAction(() -> intaking = false)
        );
    }

    public Action hoodOffsetAction() {
        return new SequentialAction(
                new InstantAction(() -> {
                })
        );
    }

    public Action shootingAction() {
        boolean far;

        shooting = true;

        return new SequentialAction(
                new InstantAction(() -> {
                    gate.open();
                    timer.reset();


                    intake.setPower(1);

                }),
                new SleepAction(0.06),
                new InstantAction(intake::start),
                new SleepAction(0.5),
                new InstantAction(() -> {
                    gateState = false;
                    shooter.closeGate();
                    time = timer.time();
                    shooting = false;
                    isFull = false;
                })
        );
    }

    public Action autonShootFar() {
        shooting = true;
        return new SequentialAction(
                new InstantAction(() -> {
                    shooter.openGate();
                    timer.reset();
                    intake.setPower(0.75);
                }),
                new SleepAction(0.06),
                new WaitUntilEmptyBB(this),
                new InstantAction(intake::start),
                new SleepAction(0.05),
                new InstantAction(() -> {
                    gateState = false;
                    gate.close();
                    time = timer.time();
                    shooting = false;
                })
        );
    }

    public Action autonShootClose() {
        shooting = true;
        return new SequentialAction(
                new InstantAction(() -> {
                    shooting = true;
                    shooter.openGate();
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