package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.CSRobot

@JoosConfig
@TeleOp
class AxonTest : CommandOpMode() {
    private val robot by robot<CSRobot>()

    companion object {
        var targetPosition = 0.0
    }

    override fun preInit() {
        schedule(true) {
            robot.outtake.armServo.position = targetPosition
        }
    }
}