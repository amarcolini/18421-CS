package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@JoosConfig
@TeleOp(group = "lol")
class LiftTuner : CommandOpMode() {
    private val robot by robot<CSRobot>()

    companion object {
        var targetPosition = 0.0
    }

    override fun preInit() {
        robot.verticalExtension.motors.resetEncoders()
        robot.verticalExtension.positionControlEnabled = true

        schedule(true) {
            robot.verticalExtension.setTargetPosition(targetPosition)
        }

        map(gamepad.p1.run { a or cross }::justActivated, ::stopOpMode)
    }
}