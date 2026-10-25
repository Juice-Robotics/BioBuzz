package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

@Config
@Autonomous(name = "BLUE FLOWER", group = "Autonomous")
public class BLUEFLOWER extends LinearOpMode {
    MecanumDrive drive;


    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-40,63, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, beginPose);

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime ejectTimer = new ElapsedTime();

        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());


        TrajectoryActionBuilder flower = drive.actionBuilder(beginPose)
                .setReversed(true)
                .splineTo(
                        new Vector2d(-43, 31),
                        Math.toRadians(315)
                )
                .waitSeconds(1.0)
                .setReversed(false)
                .splineTo(
                        new Vector2d(-60, 47),
                        Math.toRadians(150)
                )
                .splineTo(
                        new Vector2d(-64, 61),
                        Math.toRadians(100)
                )
                .waitSeconds(1.0)
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(43, 31, Math.toRadians(45)),
                        Math.toRadians(-45)
                )
                .waitSeconds(1.0)
                .setReversed(false)
                .splineTo(
                        new Vector2d(63, 31),
                        Math.toRadians(-40)
                )
                .waitSeconds(1.0)
                .setReversed(true)
                .splineTo(
                        new Vector2d(43, 31),
                        Math.toRadians(225)
                )
                .waitSeconds(1.0)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(
                        new Pose2d(-17, 61, Math.toRadians(135)),
                        Math.toRadians(135)
                )
                .waitSeconds(1.0)
                .strafeTo(new Vector2d(-43, 31))
                .waitSeconds(1.0)
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(
                        new Pose2d(23, 58, Math.toRadians(75)),
                        Math.toRadians(30)
                );;

        waitForStart();

        if (isStopRequested()) return;
        timer.reset();
        Actions.runBlocking(
                flower.build()
        );
    }
}