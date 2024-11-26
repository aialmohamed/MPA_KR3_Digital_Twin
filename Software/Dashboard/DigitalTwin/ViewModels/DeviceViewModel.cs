


using CommunityToolkit.Mvvm.Input;
using DigitalTwin.Data;

namespace DigitalTwin.ViewModels;


public partial class DeviceViewModel : PageViewModel
{
    public DeviceViewModel()
    {
        PageNames = ApplicationPageNames.Device;
    }
    [RelayCommand]
    private void AddDevice()
    {
        // Add device logic
    }
}