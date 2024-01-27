package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(group = "lol")
class DistanceSensorTest : CommandOpMode() {
    private val robot by robot<CSRobot>()

    override fun preInit() {
        schedule(true) {
            telem.addData("distance", robot.frontDistanceSensor.getDistance())
        }
    }
}