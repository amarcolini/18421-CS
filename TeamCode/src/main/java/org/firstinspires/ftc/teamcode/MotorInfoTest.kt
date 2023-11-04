package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.hardware.hMap
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx

@TeleOp
class MotorInfoTest : LinearOpMode() {
    override fun runOpMode() {
        val motor = hMap.get(DcMotorEx::class.java, "motor")
        val info = motor.motorType

        telemetry
            .addData("maxRPM", info?.maxRPM)
            .addData("gearing", info?.gearing)
            .addData("achievableMaxRPMFraction", info?.achieveableMaxRPMFraction)
            .addData("achievableMaxTicksPerSecond", info?.achieveableMaxTicksPerSecond)
            .addData("ticksPerRev", info?.ticksPerRev)
            .addData("distributor", info?.distributorInfo?.distributor)
            .addData("portNumber", motor.portNumber)
            .addData("deviceName", motor.deviceName)
            .addData("manufacturer", motor.manufacturer.name)
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            sleep(500)
        }
    }
}