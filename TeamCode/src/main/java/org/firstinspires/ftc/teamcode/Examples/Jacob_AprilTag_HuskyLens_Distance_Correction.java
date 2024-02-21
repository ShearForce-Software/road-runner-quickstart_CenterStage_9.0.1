package org.firstinspires.ftc.teamcode.Examples;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Delivery AprilTag Correction", group = "Example")
//@Disabled
public class Jacob_AprilTag_HuskyLens_Distance_Correction extends LinearOpMode {

    private DistanceSensor sensorDistance;
    private HuskyLens huskyLens;
    private double cameraOffset = 1.5;

    private double rangeToBoard = 0.0;
    private double distanceSensorError = 0.25;
    private double pixelCorrectionAmountLR = 0.0;
    private double correctionAmountFB = 0.0;
    private double centerToEdgeDistance = 0.0;
    private double pixelWidth = 0.0;
    private double distanceCorrectionLR = 0.0;

    @Override
    public void runOpMode(){

        sensorDistance = hardwareMap.get(DistanceSensor.class, "2mDistance");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens1");

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        while(!isStarted()){

        }

        waitForStart();

        while(opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            rangeToBoard = sensorDistance.getDistance(DistanceUnit.INCH) - distanceSensorError;

            telemetry.addData("range", String.format("%.01f in", rangeToBoard));

            if (blocks.length > 0){
                int xVal = blocks[0].x;
                //telemetry.addData("Center of AprilTag: ", xVal);
                pixelCorrectionAmountLR = xVal - 160;
                //telemetry.addData("Pixel Correction Value LR: ", pixelCorrectionAmountLR);
                centerToEdgeDistance = (rangeToBoard + cameraOffset) * Math.tan(Math.toRadians(30));
                //telemetry.addData("Center to Edge Distance: ","%.01f", centerToEdgeDistance);
                pixelWidth = centerToEdgeDistance / 160;
                distanceCorrectionLR = pixelCorrectionAmountLR * pixelWidth;
                telemetry.addData("Correction LF: ","%.01f", distanceCorrectionLR);
                telemetry.update();
            }else{
                telemetry.addData("No April Tags in view",0 );
                telemetry.update();
            }
        }
    }
}
