


using System;
using System.Threading.Tasks;
using Avalonia.Media;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using DigitalTwin.Data;
using DigitalTwin.OpcaClient;

namespace DigitalTwin.ViewModels;


public partial class ConnectViewModel : PageViewModel
{
    private  OpcuaClient _opcuaClient;

    [ObservableProperty]
    private string _opcuaClientStatusText = "Disconnected";

    public ConnectViewModel(Func<OpcuaClient> GetClient)
    {
        PageNames = ApplicationPageNames.Connect;
        _opcuaClient = GetClient();
    }

    [RelayCommand]
    public void ConnectToOpcuaServer()
    {
        OpcuaClientStatusText = "Connecting...";
        Task.Run(async () =>
        {
            bool value = await _opcuaClient.ConnectAsync();
            if(value)
            {
                OpcuaClientStatusText = "Connected";
            }
            else
            {
                OpcuaClientStatusText = "Failed to connect";
            }
        });

    }
    [RelayCommand]
    public void DisconnectFromOpcuaServer()
    {
        OpcuaClientStatusText =  "Disconnecting...";
        Task.Run(async () =>
        {
            await _opcuaClient.DisconnectAsync();
            OpcuaClientStatusText = "Disconnected";
        });
    }
}