package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Common base class for controls
 */
public abstract class DriveControlBase {
    private static List<Class<? extends DriveControlBase>> s_controlSystems = new ArrayList<Class<? extends DriveControlBase>>();
    private static Class<? extends DriveControlBase> s_defaultControlSystem;
    protected Map<GenericHID, boolean[]> m_lastHid = new HashMap<GenericHID, boolean[]>();

    /**
     * Gets human readable name
     * 
     * @return human readable name
     */
    public abstract String getName();

    /**
     * Poll for updates, call in a loop
     */
    public void update() {
        m_lastHid.replaceAll((k,v) -> {
            for(int i = 0; i < v.length; i++) {
                v[i] = k.getRawButton(i);
            }
            return v;
        });
    }

    /**
     * Adds a HID device to have the last value cached each time update() is called
     * 
     * @param hid HID device
     */
    public void addHid(GenericHID hid) {
        m_lastHid.put(hid, new boolean[hid.getButtonCount()]);
    }

    /**
     * Add multiple HID devices to have the last value cached each time update() is called
     * 
     * @param hids HID devices
     */
    public void addHid(GenericHID... hids) {
        for (GenericHID hid : hids) {
            addHid(hid);
        }
    }

    /**
     * Poll for control system switch, call in a loop
     * 
     * @return requested control system to switch to, or null if none.
     */
    public abstract Class<? extends DriveControlBase> changeRequested();

    /**
     * Add a control system that extends this class. Subclasses should call this method.
     * 
     * @param subclass a control system that extends this class
     */
    protected static final void addControlSystem(Class<? extends DriveControlBase> subclass) {
        addControlSystem(subclass, false);
    }

    /**
     * Add a control system that extends this class. Subclasses should call this method.
     * 
     * @param subclass a control system that extends this class
     * @param isDefault whether to make this control system default
     */
    protected static final void addControlSystem(Class<? extends DriveControlBase> subclass, boolean isDefault) {
        s_controlSystems.add(subclass);
        if (isDefault) s_defaultControlSystem = subclass;
    }

    /**
     * Gets default control system
     * 
     * @return default control system
     */
    public static final Class<? extends DriveControlBase> getDefaultControlSystem() {
        return s_defaultControlSystem;
    }

    /**
     * Gets all control systems extending this class.
     * 
     * @return control systems
     */
    public static final List<Class<? extends DriveControlBase>> getControlSystems() {
        return s_controlSystems;
    }
}