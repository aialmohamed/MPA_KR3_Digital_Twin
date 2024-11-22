
using System.Threading.Tasks;
using Avalonia.Logging;
using Avalonia.Media;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using DigitalTwin.Models;
using DigitalTwin.Services;
using HarfBuzzSharp;

namespace DigitalTwin.ViewModels;

public partial class LoginViewModel : ViewModelBase
{
    private readonly IAuthenticationService _authenticationService;
    [ObservableProperty]
    private string _username = string.Empty;

    [ObservableProperty]
    private string _password = string.Empty;

    [ObservableProperty]
    private bool _isAuthenticated = false;

    [ObservableProperty]
    private string _autheticatedUser = "Dummy";

    [ObservableProperty]
    private string _operationResult = string.Empty;

    [ObservableProperty]
    private IBrush _operationResultColor;
    public LoginViewModel(IAuthenticationService authenticationService)
    {
        _authenticationService = authenticationService;
    }
    [RelayCommand]
    private async Task Register()
    {
        
    }

    [RelayCommand]
    private async Task Login()
    {
        try
        {
                var isAuthenticated  = await _authenticationService.AuthenticateUserAsync(Username, Password);
                if (isAuthenticated)
                {
                    IsAuthenticated = true;
                    AutheticatedUser = Username;
                    OperationResult = "Login successful!";
                    OperationResultColor = Brushes.Green;
                }
                else
                {
                    IsAuthenticated = false;
                    AutheticatedUser = "Not authenticated";
                    OperationResult = "Invalid username or password.";
                    OperationResultColor = Brushes.Red;
                }
                 
        }catch (System.Exception ex)
        {
            IsAuthenticated = false;
            AutheticatedUser = "Not authenticated";
            OperationResult = "An error occurred during login.";
            OperationResultColor = Brushes.Yellow;
            Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error during login: {ex.Message}");
        }
    }

}