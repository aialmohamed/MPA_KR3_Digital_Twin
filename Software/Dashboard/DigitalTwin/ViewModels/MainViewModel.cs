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
using DigitalTwin.OpcaClient;
using System.ComponentModel;
using System.Threading.Tasks;
using DigitalTwin.Container;
using Avalonia.Logging;
using System.IO;
namespace DigitalTwin.ViewModels;

public partial class MainViewModel : ViewModelBase
{
    private readonly OpcuaClient _opcuaClient;
    private  SystemContainer _systemContainer;
    private PageFactory _pageFactory;
    private readonly Func<UserSession> _userSessionFactory;
    public UserSession CurrentUserSession => _userSessionFactory();

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(IsAuthenticated))]
    [NotifyPropertyChangedFor(nameof(AuthenticatedUser))]
    private PageViewModel _currentPage;
    


    public MainViewModel(PageFactory pageFactory , Func<UserSession> userSessionFactory,Func<ProjectPaths> GetProjectPaths)
    {
        _systemContainer = new SystemContainer(GetProjectPaths);
        Task.Run(async () =>
        {
            try
            {
                string imageName = "ibo311/kr3r540_digital_twin:v1.0";
                string dockerFilePath = _systemContainer.DockerFilePath; 

                Console.WriteLine($"Ensuring Docker image '{imageName}' exists...");
                await _systemContainer.EnsureDockerImageExistsAsync(imageName, dockerFilePath);

                Console.WriteLine($"Launching Container...");
                await _systemContainer.LaunchDockerContainerInBackgroundAsync();

                Console.WriteLine($"Waiting for the OPC UA server on port 4840...");
                bool isServerReady = await _systemContainer.WaitForOpcUaServerAsync("opc.tcp://localhost:4840");

                if (isServerReady)
                {
                    Console.WriteLine("The OPC UA server is ready.");
                }
                else
                {
                    Console.WriteLine("The server failed to start within the timeout period.");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"An error occurred: {ex.Message}");
            }
        });
        _opcuaClient = new OpcuaClient();
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
        GoToHome();
    }
    
    public bool IsAuthenticated => CurrentUserSession.IsAuthenticated;
    public string AuthenticatedUser => CurrentUserSession.Username;


    // Design-time constructor (shell be removed in production)
    public MainViewModel()
    {
        if (Design.IsDesignMode)
        {
            
           // CurrentPage = new HomeViewModel();
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
    [RelayCommand]
    private void GoToLaunch()
    {
        CurrentPage = _pageFactory.GetPageViewModel(ApplicationPageNames.Launch);
    }
    [RelayCommand]
    private void Connect()
    {
        
        Task.Run(async () =>
        {
            try
            {
                Console.WriteLine($"Removing Conatiner");
                await _systemContainer.StopAndRemoveDockerContainerAsync("kr3r540_digital_twin");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"An error occurred while checking Docker container: {ex.Message}");
            }
        });
    }
    [RelayCommand]
    private async Task Start()
    {
        await _opcuaClient.CallSystemMethodAsync(_opcuaClient.Session,"ns=2;s=ControlMethods","ns=2;s=LaunchRos2Simulation");
    }
    [RelayCommand]
    private async Task Settings()
    {
        await _opcuaClient.CallSystemMethodAsync(_opcuaClient.Session,"ns=2;s=ControlMethods","ns=2;s=LaunchRos2Simulation");
    }

}
