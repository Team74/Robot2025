package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class BotLED {

  //This code was copied from online, so some of the comments are not from me. 
  //This class controls the RGBs on the bot. 

  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;

  BotLED(int PWMport) {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(PWMport);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(69);// this is not bc funny number, just how many LEDs there were
    m_led.setLength(m_ledBuffer.getLength());
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  void setColor(int r, int g, int b) {
    //For each LED, set the color, go to the next LED
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      //The LED strip was BRG instead of RGB for some reason
      m_ledBuffer.setRGB(i, b / 6, r / 6, g / 6);
    }

    m_led.setData(m_ledBuffer);
  }

  //Sets the LEDs to rainbow. Copied code
  void rainbow() {
    int m_rainbowFirstPixelHue = 0;
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }

    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;

    m_led.setData(m_ledBuffer);

  }

}
