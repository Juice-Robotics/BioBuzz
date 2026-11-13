
package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "kavinbluegarden", group = "Autonomous")
public class KavinBlueGarden extends LinearOpMode {

    private MecanumDrive drive;

    private static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(
                -59.5,
                30,
                rad(-150)
        );

        drive = new MecanumDrive(hardwareMap, beginPose);

        ElapsedTime timer = new ElapsedTime();

        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(
                telemetry,
                dash.getTelemetry()
        );

        TrajectoryActionBuilder blueGardenKavin =
                drive.actionBuilder(beginPose)


                        .setTangent(rad(90))
                        .splineToLinearHeading(
                                new Pose2d(-62, 60, rad(150)),
                                rad(90)
                        )

                        .setTangent(rad(0))
                        .splineToLinearHeading(
                                new Pose2d(45, 52, rad(0)),
                                rad(0)
                        )

                        .setTangent(rad(-90))
                        .splineToLinearHeading(
                                new Pose2d(56, 20, rad(0)),
                                rad(-90)
                        )


                        .setTangent(rad(145))
                        .splineToLinearHeading(
                                new Pose2d(36, 50, rad(90)),
                                rad(90)
                        )
                        .lineToY(62)

                        .setTangent(rad(-140))
                        .splineToLinearHeading(
                                new Pose2d(-36, 38, rad(130)),
                                rad(-150)
                        )

                        .waitSeconds(2)

                        .setTangent(rad(60))
                        .splineToLinearHeading(
                                new Pose2d(-24, 45, rad(90)),
                                rad(90)
                        )
                        .lineToY(58)

                        .waitSeconds(1)

                        .setTangent(rad(-120))
                        .splineToLinearHeading(
                                new Pose2d(-36, 38, rad(130)),
                                rad(-150)
                        )

                        .waitSeconds(1)

                        .setTangent(rad(-135))
                        .splineToLinearHeading(
                                new Pose2d(-62, 24, rad(-90)),
                                rad(-90)
                        )
                        .lineToY(12)

                        .setTangent(rad(90))
                        .splineToLinearHeading(
                                new Pose2d(-48, 48, rad(0)),
                                rad(0)
                        )

                        .splineToLinearHeading(
                                new Pose2d(36, 48, rad(0)),
                                rad(0)
                        )

                        .setTangent(rad(-90))
                        .splineToLinearHeading(
                                new Pose2d(56, 20, rad(0)),
                                rad(-90)
                        )

                        .setTangent(rad(90))
                        .splineToLinearHeading(
                                new Pose2d(48, 55, rad(-80)),
                                rad(90)
                        );


        waitForStart();

        if (isStopRequested()) {
            return;
        }

        timer.reset();

        Actions.runBlocking(
                blueGardenKavin.build()
        );


    }
}
