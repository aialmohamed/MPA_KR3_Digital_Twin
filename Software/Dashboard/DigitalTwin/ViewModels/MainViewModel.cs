using Avalonia.Automation;
using Avalonia.Controls;
using Avalonia.Data.Converters;
using Avalonia.Metadata;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.DependencyInjection;
using DigitalTwin.Data;
using DigitalTwin.Factories;
using Microsoft.Extensions.DependencyInjection;

namespace DigitalTwin.ViewModels;

public partial class MainViewModel : ViewModelBase
{
    private PageFactory _pageFactory;


    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(IsAuthenticated))]
    [NotifyPropertyChangedFor(nameof(AuthenticatedUser))]
    private PageViewModel _currentPage;
    
    public bool IsAuthenticated => (CurrentPage as IAuthenticationAware)?.IsAuthenticated ?? false;
    public string AuthenticatedUser => (CurrentPage as IAuthenticationAware)?.AuthenticatedUser ?? string.Empty;


    public MainViewModel(PageFactory pageFactory)
    {
        _pageFactory = pageFactory;
        GoToLoginPage();
    }


    // Design-time constructor (shell be removed in production)
    public MainViewModel()
    {
        if (Design.IsDesignMode)
        {

            //CurrentPage = new LoginViewModel();
        }
    }

    public void GoToLoginPage()
    {
        var page = _pageFactory.GetPageViewModel(ApplicationPageNames.Login);

        if (page is IAuthenticationAware authAwarePage)
        {
            authAwarePage.PropertyChanged += (sender, args) =>
            {
                if (args.PropertyName == nameof(IAuthenticationAware.IsAuthenticated) ||
                    args.PropertyName == nameof(IAuthenticationAware.AuthenticatedUser))
                {
                    OnPropertyChanged(nameof(IsAuthenticated));
                    OnPropertyChanged(nameof(AuthenticatedUser));
                }
            };
        }

        CurrentPage = page;

        
    }
}
