using CommunityToolkit.Mvvm.ComponentModel;
using Microsoft.Extensions.DependencyInjection;

namespace DigitalTwin.ViewModels;

public partial class MainViewModel : ViewModelBase
{
    [ObservableProperty]
    private ViewModelBase _currentPage;

    private readonly LoginViewModel _loginViewModel = new ();
     // addiing a service collection for DI 
     



    public MainViewModel()
    {
        CurrentPage = _loginViewModel;

    }

}
