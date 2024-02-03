/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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
