
package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

@TeleOp(name = "MAIN")
@Config
public class MAIN extends LinearOpMode {

    double oldTime = 0;

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

        while (opModeInInit() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        robot.updateAim();

        while (opModeIsActive() && !isStopRequested()) {

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            TelemetryPacket packet = new TelemetryPacket();

            robot.update();
            robot.updateAim();

            if (gamepad1.leftBumperWasPressed()) {
                if (!robot.shooting) {
                    actionsQueue.add(robot.shootingAction());
                }
            }

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            robot.setDrivePower(x, y, rx);

            Iterator<Action> it = actionsQueue.iterator();
            while (it.hasNext()) {
                Action action = it.next();
                action.preview(packet.fieldOverlay());
                if (!action.run(packet)) {
                    it.remove();
                }
            }

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;

            telemetry.addData("LOOPTIME: ", frequency);
            telemetry.update();
        }
    }
}
