
using CommunityToolkit.Mvvm.ComponentModel;
using DigitalTwin.Data;

namespace DigitalTwin.ViewModels;

public partial class PageViewModel : ViewModelBase
{
    [ObservableProperty]
    private ApplicationPageNames _pageNames;

    [ObservableProperty]
    private bool _isAuthenticated = false;

    [ObservableProperty]
    private string _authenticatedUser = string.Empty;
}