using System;
using Avalonia.Automation;
using Avalonia.Controls;
using Avalonia.Data.Converters;
using Avalonia.Metadata;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.DependencyInjection;
using CommunityToolkit.Mvvm.Input;
using DigitalTwin.Data;
using DigitalTwin.Factories;
using DigitalTwin.Models;
using Microsoft.Extensions.DependencyInjection;

namespace DigitalTwin.ViewModels;

public partial class MainViewModel : ViewModelBase
{
    private PageFactory _pageFactory;
    private readonly Func<UserSession> _userSessionFactory;
    public UserSession CurrentUserSession => _userSessionFactory();

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(IsAuthenticated))]
    [NotifyPropertyChangedFor(nameof(AuthenticatedUser))]
    private PageViewModel _currentPage;
    


    public MainViewModel(PageFactory pageFactory , Func<UserSession> userSessionFactory)
    {
        _userSessionFactory = userSessionFactory;
        _pageFactory = pageFactory;
        // Subscribe to property changes in UserSession
        CurrentUserSession.PropertyChanged += (sender, args) =>
        {
            if (args.PropertyName == nameof(UserSession.IsAuthenticated) ||
                args.PropertyName == nameof(UserSession.Username))
            {
                OnPropertyChanged(nameof(IsAuthenticated));
                OnPropertyChanged(nameof(AuthenticatedUser));
            }
        };
    }
    
    public bool IsAuthenticated => CurrentUserSession.IsAuthenticated;
    public string AuthenticatedUser => CurrentUserSession.Username;


    // Design-time constructor (shell be removed in production)
    public MainViewModel()
    {
        if (Design.IsDesignMode)
        {

            //CurrentPage = new RegisterViewModel();
        }
    }

    [RelayCommand]
    private void GoToRegister()
    {
        CurrentPage = _pageFactory.GetPageViewModel(ApplicationPageNames.Register);
    }
    [RelayCommand]
    private void GoToLogin()
    {
        CurrentPage = _pageFactory.GetPageViewModel(ApplicationPageNames.Login);

    }
    [RelayCommand]
    private void Logout()
    {
        CurrentUserSession.ClearSession();
        GoToLogin();

    }
    [RelayCommand]
    private void GoToHome()
    {
        CurrentPage = _pageFactory.GetPageViewModel(ApplicationPageNames.Home);
    }

}
