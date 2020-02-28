package frc.robot.utils;

/**
 * Utility class for JNI when representing a pointer
 */
public class Pointer {
    public final long address;
    public Pointer(long ptr) {
        address = ptr;
    }
}