package frc.robot.utils.pixycam;

public enum PixyError {
    PIXY_RESULT_OK((byte) 0),               // no error
    PIXY_RESULT_ERROR((byte) -1),           // general error
    PIXY_RESULT_BUSY((byte) -2),            // pixy has no new data, used in polling mode
    PIXY_RESULT_CHECKSUM_ERROR((byte) -3),  // data packet has checksum error
    PIXY_RESULT_TIMEOUT((byte) -4),         // pixy data took too long to return result
    PIXY_RESULT_BUTTON_OVERRIDE((byte) -5), // user is interacting with the button
    PIXY_RESULT_PROG_CHANGING((byte) -6),   // program is changing (undocumented)
    JNI_RESULT_ARGUMENT_OUT_OF_RANGE((byte) -128);   // argument(s) passed out of range

    public final byte id;

    PixyError(byte errorId) {
        id = errorId;
    }
}