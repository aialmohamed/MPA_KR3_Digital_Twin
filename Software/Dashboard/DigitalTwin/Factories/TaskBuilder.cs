

using DigitalTwin.Data;

public class TaskBuilder
{
    private double _x;
    private double _y;
    private double _z;
    private double _roll;
    private double _pitch;
    private double _yaw;
    private int _gripper;

    public TaskBuilder SetX(double x)
    {
        _x = x;
        return this;
    }

    public TaskBuilder SetY(double y)
    {
        _y = y;
        return this;
    }

    public TaskBuilder SetZ(double z)
    {
        _z = z;
        return this;
    }

    public TaskBuilder SetRoll(double roll)
    {
        _roll = roll;
        return this;
    }

    public TaskBuilder SetPitch(double pitch)
    {
        _pitch = pitch;
        return this;
    }

    public TaskBuilder SetYaw(double yaw)
    {
        _yaw = yaw;
        return this;
    }

    public TaskBuilder SetGripper(bool gripper)
    {
        // Convert bool to int (true = 1, false = 0)
        _gripper = gripper ? 1 : 0;
        return this;
    }

    public TaskData Build()
    {
        // Construct the final TaskData object
        return new TaskData(_x, _y, _z, _roll, _pitch, _yaw, _gripper);
    }
}
