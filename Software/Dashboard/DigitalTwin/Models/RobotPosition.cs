



using System;
using System.Threading.Tasks;
using Avalonia.Logging;
using CommunityToolkit.Mvvm.Input;
using DigitalTwin.OpcaClient;

namespace DigitalTwin.Models;


public class RobotPosition
{
    public string XCoordinate { get; set; }
    public string YCoordinate { get; set; }
    public string ZCoordinate { get; set; }
    public string Roll { get; set; }
    public string Pitch { get; set; }
    public string Yaw { get; set; }
    public string Gripper { get; set; }
    public RobotPosition()
    {
       
    }
}