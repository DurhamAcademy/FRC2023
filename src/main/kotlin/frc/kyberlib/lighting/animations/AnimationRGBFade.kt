package frc.kyberlib.lighting.animations

import frc.kyberlib.math.units.extensions.Time
import java.awt.Color

class AnimationRGBFade(val cycle: Time, enableTransparency: Boolean = false, condition: ()->Boolean = { true }) : LEDAnimation(condition, enableTransparency) {
    var buffer = mutableListOf<Color>()
    override fun getBuffer(time: Time, length: Int): List<Color> {
        val color = Color.getHSBColor(((time % cycle) / cycle).toFloat() % 1F, 1F, 1F)
        if (length != buffer.size) {
            buffer = Array<Color>(length) {
                color
            }.toMutableList()
        } else {
            for (i in 0 until length) {
                buffer[i] = color
            }
        }
        return buffer
    }
}
