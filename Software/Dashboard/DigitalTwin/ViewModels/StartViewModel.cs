


using System;
using CommunityToolkit.Mvvm.ComponentModel;
using DigitalTwin.Data;
using DigitalTwin.OpcaClient;
namespace DigitalTwin.ViewModels;


public partial class StartViewModel : PageViewModel
{
    [ObservableProperty]
    private string _xCoords_real;
    [ObservableProperty]
    private string _yCoords_real;
    [ObservableProperty]
    private string _zCoords_real;
    [ObservableProperty]
    private string _roll_real;
    [ObservableProperty]
    private string _pitch_real;
    [ObservableProperty]
    private string _yaw_real;
    [ObservableProperty]
    private string _gripper_real;

    [ObservableProperty]
    private string _xCoords_sim;
    [ObservableProperty]
    private string _yCoords_sim;
    [ObservableProperty]
    private string _zCoords_sim;
    [ObservableProperty]
    private string _roll_sim;
    [ObservableProperty]
    private string _pitch_sim;
    [ObservableProperty]
    private string _yaw_sim;
    [ObservableProperty]
    private string _gripper_sim;
    
    private readonly OpcuaClient _opcaClient;
    public StartViewModel(Func<OpcuaClient> getOpcuaClient)
    {
        PageNames = ApplicationPageNames.Start;
        _opcaClient = getOpcuaClient();
    }
}   