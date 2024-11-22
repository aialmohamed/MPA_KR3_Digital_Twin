using Avalonia.Automation;
using Avalonia.Controls;
using Avalonia.Data.Converters;
using Avalonia.Metadata;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.DependencyInjection;
using Microsoft.Extensions.DependencyInjection;

namespace DigitalTwin.ViewModels;

public partial class MainViewModel : ViewModelBase
{
    [ObservableProperty]
    private ViewModelBase _currentPage;

    [ObservableProperty]
    private LoginViewModel _loginViewModel;

    public MainViewModel(LoginViewModel loginViewModel)
    {
        LoginViewModel = loginViewModel;
        CurrentPage = loginViewModel;
    }

    // Design-time constructor (shell be removed in production)
    public MainViewModel()
    {
        if (Design.IsDesignMode)
        {
            //CurrentPage = new LoginViewModel();
        }
    }
}
