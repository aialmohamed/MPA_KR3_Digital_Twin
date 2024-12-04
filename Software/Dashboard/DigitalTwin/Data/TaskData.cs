

namespace DigitalTwin.Data;


public class TaskData
{
    public double X { get; private set; }
    public double Y { get; private set; }
    public double Z { get; private set; }
    public double Roll { get; private set; }
    public double Pitch { get; private set; }
    public double Yaw { get; private set; }
    public int Gripper { get; private set; } 

    public TaskData(double x, double y, double z, double roll, double pitch, double yaw, int gripper)
    {
        X = x;
        Y = y;
        Z = z;
        Roll = roll;
        Pitch = pitch;
        Yaw = yaw;
        Gripper = gripper;
    }

    public override string ToString()
    {
        return $"TaskData: X={X}, Y={Y}, Z={Z}, Roll={Roll}, Pitch={Pitch}, Yaw={Yaw}, Gripper={Gripper}";
    }
}
