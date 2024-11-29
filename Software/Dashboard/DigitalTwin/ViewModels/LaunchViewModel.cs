


using CommunityToolkit.Mvvm.Input;
using DigitalTwin.Data;

namespace DigitalTwin.ViewModels;

// Starting the Docker Container and check status of docker 
public partial class LaunchViewModel : PageViewModel
{
    public LaunchViewModel()
    {
        PageNames = ApplicationPageNames.Launch;
    }
    [RelayCommand]
    private void AddDevice()
    {
        // Add device logic
    }
}