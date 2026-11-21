
package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "lutmaker")
@Config
public class LUTMaker extends LinearOpMode {
    double oldTime = 0;
    boolean oldRTrigger = false;
    boolean oldLTrigger = false;
    public static double flywheelRPM;
    public static double hoodAngle;
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        Robot robot = new Robot(hardwareMap, false);


        List<Action> actionsQueue = new ArrayList<>();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            TelemetryPacket packet = new TelemetryPacket();


            if (gamepad1.leftBumperWasPressed() && !robot.shooting) {
                actionsQueue.add(robot.shootingAction());
            }

            if (gamepad1.rightBumperWasPressed() && !robot.shooting && !robot.intaking) {
                actionsQueue.add(robot.startIntakeAndTransfer(gamepad1));
            }

            if (gamepad1.triangleWasPressed()) {
                actionsQueue.add(
                        new SequentialAction(
                                new InstantAction(robot.intake::reverse),
                                new SleepAction(0.5),
                                robot.startIntakeAndTransfer(gamepad1)
                        )
                );
            }

            if (gamepad1.crossWasPressed()) {
                if (robot.intaking) {
                    actionsQueue.add(robot.stopIntakeAndTransfer());
                }
            }
            robot.pinpoint.update();
            robot.shooter.update();

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            robot.setDrivePower(x, y, rx);

            List<Action> newActions = new ArrayList<>();
            for (Action action : actionsQueue) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            actionsQueue = newActions;


            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;


            telemetry.addData("LOOPTIME: ", frequency);
            telemetry.addData("gate", robot.gateState);
            telemetry.addData("CycleTime", Robot.time);
            Pose2D pose = robot.pinpoint.getPosition();
            Pose2D tPose = new Pose2D(DistanceUnit.INCH, -60, 60, AngleUnit.RADIANS, 0);
            double dx = tPose.getX(DistanceUnit.INCH) - pose.getX(DistanceUnit.INCH);
            double dy = tPose.getY(DistanceUnit.INCH) - pose.getY(DistanceUnit.INCH);

            double distance = Math.hypot(dx, dy);
            telemetry.addData("dx", dx);
            telemetry.addData("dy", dy);
            telemetry.addData("dist", distance);

//            telemetry.addData("offset", robot.turretManualOffset);



            telemetry.update();
            dash.sendTelemetryPacket(packet);
        }
    }
}