from wpilib import Color, AddressableLED, Timer
import wpilib
import commands2
import math

kLedBuffer = 40
kBreathingSpeed = 3

class led(commands2.Subsystem):
    def __init__(self):
        """ 
        Initializes the led strip and sets the relevant settings for the leds. 
        """
        self.led = AddressableLED(0)
        self.timer = Timer()

        self.led.setLength(kLedBuffer)
        self.rainbowFirstPixelHue = 0

        self.ledData = [AddressableLED.LEDData() for _ in range(kLedBuffer)]

        self.led.setData(self.ledData)
        self.timer.start()
        self.led.start()
        

        super().__init__()

    def rainbow(self):
        # For every pixel
        for i in range(kLedBuffer):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            hue = (self.rainbowFirstPixelHue + (i * 180 / kLedBuffer)) % 180

            # Set the value
            self.ledData[i].setHSV(int(hue), 255, 128)

        # Increase by to make the rainbow "move"
        self.rainbowFirstPixelHue += 1

        # Check bounds
        self.rainbowFirstPixelHue %= 180

        self.led.setData(self.ledData)

    def red(self):
        for i in range(kLedBuffer):
            self.ledData[i].setLED(Color.kRed)

        self.led.setData(self.ledData)

    def green(self):
        for i in range(kLedBuffer):
            self.ledData[i].setLED(Color.kGreen)

        self.led.setData(self.ledData)

    def off(self):
        for i in range(kLedBuffer):
            self.ledData[i].setLED(Color.kBlack)

        self.led.setData(self.ledData)

    def greenBlink(self):
        if self.timer.get() < 0.5:
            self.green()
        else:
            self.off()

        if self.timer.get() > 1.0:
            self.timer.reset()
    
    def redBlink(self):
        if self.timer.get() < 0.5:
            self.red()
        else:
            self.off()

        if self.timer.get() > 1.0:
            self.timer.reset()

    def greenBreathing(self):
        brightness = int((math.sin(self.timer.get()) + 1) / kBreathingSpeed * 255)

        for i in range(kLedBuffer):
            self.ledData[i].setRGB(0, brightness, 0)
        
        self.led.setData(self.ledData)

            
    def white(self):
        for i in range(kLedBuffer):
            self.ledData[i].setLED(Color.kWhite)

        self.led.setData(self.ledData)