package frc.robot.utils.pixycam;

public class Block {
    public final int m_signature;
    public final int m_x;
    public final int m_y;
    public final int m_width;
    public final int m_height;
    public final int m_angle;
    public final int m_index;
    public final int m_age;

    public Block(int signature, int x, int y, int width, int height, int angle, int index, int age) {
        m_signature = signature;
        m_x = x;
        m_y = y;
        m_width = width;
        m_height = height;
        m_angle = angle;
        m_index = index;
        m_age = age;
    }
}