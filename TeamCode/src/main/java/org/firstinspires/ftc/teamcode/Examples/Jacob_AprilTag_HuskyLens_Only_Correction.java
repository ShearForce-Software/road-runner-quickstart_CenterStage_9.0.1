package org.firstinspires.ftc.teamcode.Examples;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Delivery AprilTag Only Correction", group = "Example")
//@Disabled
public class Jacob_AprilTag_HuskyLens_Only_Correction extends LinearOpMode {

    HuskyLens huskyLens;
    double hl_rangeToBoard = 0.0;
    double pixelCorrectionAmountLR = 0.0;
    double pixelWidth_HL = 0.0;
    double distanceCorrectionLR_HL = 0.0;
    double hl_halfScreenWidth = 0.0;

    @Override
    public void runOpMode(){

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

            if (blocks.length > 0){
                double xVal = blocks[0].x;
                pixelCorrectionAmountLR = xVal - 160;

                double xWidth = blocks[0].width;
                pixelWidth_HL = 2/xWidth;
                distanceCorrectionLR_HL = pixelCorrectionAmountLR * pixelWidth_HL;

                telemetry.addData("AprilTag width: ", xWidth);
                telemetry.addData("pixel width HL: ", "%.04f", pixelWidth_HL);

                hl_halfScreenWidth = pixelWidth_HL * 160;
                hl_rangeToBoard = (pixelWidth_HL * 160) / Math.tan(Math.toRadians(30));

                telemetry.addData("HL range to board", "%.01f in", hl_rangeToBoard);
                telemetry.addData("HL Half Screen Width", "%.01f in", hl_halfScreenWidth);
                telemetry.addData("Correction LR: ","%.01f", distanceCorrectionLR_HL);
                telemetry.update();

            }else{
                telemetry.addData("No April Tags in view",0 );
                telemetry.update();
            }
        }
    }
}
