package frc.robot.hardware.pixycam;

import java.io.IOException;

import frc.robot.utils.Pointer;
import frc.robot.utils.pixycam.Block;
import frc.robot.utils.pixycam.PixyError;

/**
 * This class is for interfacing with the pixycam, and internally operates on JNI
 */
public class Pixy2 {
    public final Pointer pixy;
    public final Pixy2CCC ccc;
    public final Pixy2Video video;

    /**
     * Interface with the pixycam
     * 
     * @throws IOException error when initializing pixycam
     */
    public Pixy2() throws IOException {
        pixy = new Pointer(getPixy());
        byte result = initPixy();
        if (result < 0) {
            throw new IOException("Error initializing pixycam. Code: " + getErrorName(result) + " (" + result + ")");
        }
        ccc = new Pixy2CCC();
        video = new Pixy2Video();
    }
    
    private class Pixy2CCC {
        public Block[] blocks;

        /**
         * returned structure: [retval, block0data0, ..., block0data7, block1data0, ...]
         */
        private native int[] getBlocks(long pixyPtr, boolean wait, int sigmap, int maxBlocks);

        public byte getBlocks(boolean wait, int sigmap, int maxBlocks) {
            int[] result = getBlocks(pixy.address, wait, sigmap, maxBlocks);
            blocks = new Block[(result.length - 1) / 8];
            for (int i = 1; i < result.length; i += 8) {
                blocks[(i - 1) / 8] = new Block(result[i], result[i+1], result[i+2], result[i+3], result[i+4], result[i+5], result[i+6], result[i+7]);
            }
            return (byte) result[0];
        }

        public byte getBlocks(boolean wait, int sigmap) { return getBlocks(wait, sigmap, 255); }

        public byte getBlocks(boolean wait) { return getBlocks(wait, 255, 255); }

        public byte getBlocks() { return getBlocks(true, 255, 255); }
    }

    // TODO: implement pixy.line

    private class Pixy2Video {
        private native int[] getRGB(long pixyPtr, int x, int y, boolean saturate);

        public int[] getRGB(int x, int y, boolean saturate) { return getRGB(pixy.address, x, y, saturate); }

        public int[] getRGB(int x, int y) { return getRGB(x, y, true); }
    }

    /**
     * Gets a reference to the pixycam
     * 
     * @return pointer to pixycam object
     */
    private native long getPixy();

    /**
     * Initializes pixycam
     * 
     * @param pixyPtr pointer to pixycam object
     * @param arg pixycam args (default 0x80000000)
     * @return result code; <0 if error
     */
    private native byte initPixy(long pixyPtr, long arg);

    private byte initPixy(long pixyPtr) { return initPixy(pixyPtr, 0x80000000); }

    private byte initPixy() { return initPixy(pixy.address); }

    private native byte changeProg(long pixyPtr, String prog);

    /**
     * Switches the pixycam's program
     * 
     * @param prog program name; can be partial string if it is unique
     * @return result code; <0 if error
     */
    public byte changeProg(String prog) { return changeProg(pixy.address, prog); };

    private native byte setCameraBrightness(long pixyPtr, int brightness);

    /**
     * Sets relative exposure of pixycam's image sensor
     * 
     * @param brightness relative exposure, [0,255]
     * @return result code; <0 if error
     */
    public byte setCameraBrightness(int brightness) { return setCameraBrightness(pixy.address, brightness); }

    private native byte setLED(long pixyPtr, int r, int g, int b);

    /**
     * Sets led color of pixycam, overriding settings
     * 
     * @param r red [0,255]
     * @param g green [0,255]
     * @param b blue [0,255]
     * @return result code; <0 if error
     */
    public byte setLED(int r, int g, int b) { return setLED(pixy.address, r, g, b); }

    private native byte setLamp(long pixyPtr, boolean upper, boolean lower);

    /**
     * Turn on/off pixycam's integrated light source
     * 
     * @param upper set two white LEDs along top edge
     * @param lower set RGB LED to all on/off
     * @return result code; <0 if error
     */
    public byte setLamp(boolean upper, boolean lower) { return setLamp(pixy.address, upper, lower); }

    private native int getFrameWidth(long pixyPtr);

    public int getFrameWidth() { return getFrameWidth(pixy.address); }

    private native int getFrameHeight(long pixyPtr);

    public int getFrameHeight() { return getFrameHeight(pixy.address); }

    private native int getFPS(long pixyPtr);

    public int getFPS() { return getFPS(pixy.address); }

    /**
     * Gets pixy error name associated with id
     * 
     * @param errorId pixy error id
     * @return pixy error name
     */
    public final static String getErrorName(byte errorId) {
        for (PixyError err : PixyError.values()) {
            if (err.id == errorId) {
                return err.name();
            }
        }
        return "RESULT_UNKNOWN";
    }

    static {
        System.load("/home/lvuser/lib/libRobotJni.so");
    }
}