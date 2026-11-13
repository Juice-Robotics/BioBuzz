package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.actions.LoopAction;
//import org.firstinspires.ftc.teamcode.actions.WaitUntilBB;
//import org.firstinspires.ftc.teamcode.actions.WaitUntilBBFarAuto;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
//import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;
//import org.firstinspires.ftc.teamcode.util.hardware.GoBildaLEDIndicator;
//import org.firstinspires.ftc.teamcode.util.helpers.MatchStorage;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

@Config
@Autonomous(name = "qwertyonetwo", group = "Autonomous")
public class otherBlueGarden extends LinearOpMode {
  //  Robot robot;
    MecanumDrive drive;

    public void runOpMode() {
       // MatchStorage.setAllianceColor(AllianceColor.BLUE); // Changed to BLUE
        // Mirrored Y and Heading (90 -> -90)
        Pose2d beginPose = new Pose2d(64.772, -6.228, Math.toRadians(-90));
        Pose2d shootPose = new Pose2d(61.75, -18.25, Math.toRadians(-90));

        drive = new MecanumDrive(hardwareMap, beginPose);
        ((PinpointLocalizer) drive.localizer).driver.recalibrateIMU();
       // robot = new Robot(hardwareMap, true);

        // Inverted manual offset for mirrored turret logic
        TrajectoryActionBuilder spike1 =  drive.actionBuilder(new Pose2d((70-6.25),(-15.25), Math.toRadians(-90)))
                .setTangent(Math.toRadians(180))

                .setTangent(Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(36, -30))
                .strafeToConstantHeading(new Vector2d(36, -48))
                .strafeToLinearHeading(new Vector2d(60, -10), Math.toRadians(-90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(64, -57), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(60, -10), Math.toRadians(-90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(60, -57), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(60, -10), Math.toRadians(-90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(56, -57), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(60, -10), Math.toRadians(-90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(52, -57), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(60, -10), Math.toRadians(-180))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(30, -62.8), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(60, -10), Math.toRadians(-180))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(30, -62.8), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(30, -40), Math.toRadians(-180))
                ;

        waitForStart();
        Actions.runBlocking(new SequentialAction(spike1.build()));
    }
}
