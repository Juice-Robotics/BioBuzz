package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Tip Detector", group = "Vision")
public class tipdetector extends LinearOpMode {

    private Limelight3A limelight;
    private static final double TIPPED_PITCH_THRESHOLD = 20.0;
    // need to test and change this
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Ready to scan Hive.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            boolean tagFound = false;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> detections = result.getFiducialResults();

                for (LLResultTypes.FiducialResult tag : detections) {
                    Pose3D pose = tag.getTargetPoseCameraSpace();
                    if (pose != null) {
                        tagFound = true;

                        int tagId = tag.getFiducialId();
                        double hivePitch = pose.orientation.getPitch(AngleUnit.DEGREES);
                        double distanceIn = pose.getPosition().z * 39.3701;

                        String hiveState = "Up";
                        if (hivePitch > TIPPED_PITCH_THRESHOLD) {
                            hiveState = "TIPPED FORWARD";
                        } else if (hivePitch < -TIPPED_PITCH_THRESHOLD) {
                            hiveState = "TIPPED BACKWARD";
                        }

                        telemetry.addData("Detected Tag ID", tagId);
                        telemetry.addData("Hive State", hiveState);
                        telemetry.addData("Hive Pitch (deg)", "%.2f", hivePitch);
                        telemetry.addData("Distance to Hive (in)", "%.2f", distanceIn);
                        break;
                    }
                }
            }

            if (!tagFound) {
                telemetry.addData("Hive Detection", "No Hive AprilTag visible.");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
