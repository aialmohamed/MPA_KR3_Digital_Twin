

using CommunityToolkit.Mvvm.ComponentModel;
using DigitalTwin.Data;

namespace DigitalTwin.ViewModels;


public partial class HomeViewModel : PageViewModel
{
    private readonly HomeInfo _homeInfo;

    [ObservableProperty]
    private string _osInfo;
    [ObservableProperty]
    private string _dockerInfo;

    [ObservableProperty]
    private string _dtImageInfo;
    public HomeViewModel(HomeInfo homeInfo)
    {
        _homeInfo = homeInfo;
        PageNames = ApplicationPageNames.Home;
        SetHomeInfo();
    }

    public void SetHomeInfo()
    {
        OsInfo = _homeInfo.Os_Name;
        DockerInfo = _homeInfo.IsDockerInstalled;
        DtImageInfo = _homeInfo.IsDigitalTwinImagePresent;
    }


}