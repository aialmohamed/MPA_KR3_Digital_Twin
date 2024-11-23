
using System.Threading.Tasks;
using Avalonia.Logging;
using Avalonia.Media;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using DigitalTwin.Data;
using DigitalTwin.Models;
using DigitalTwin.Services;
using HarfBuzzSharp;

namespace DigitalTwin.ViewModels;

public partial class LoginViewModel : PageViewModel,IAuthenticationAware
{
    private readonly IAuthenticationService _authenticationService;
    [ObservableProperty]
    private string _username = string.Empty;

    [ObservableProperty]
    private string _password = string.Empty;

    [ObservableProperty]
    private string _operationResult = string.Empty;

    [ObservableProperty]
    private IBrush _operationResultColor = Brushes.Black;
    public LoginViewModel(IAuthenticationService authenticationService)
    {
         PageNames = ApplicationPageNames.Login;
         
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
                    AuthenticatedUser = Username;
                    OperationResult = "Login successful!";
                    OperationResultColor = Brushes.Green;
                }
                else
                {
                    IsAuthenticated = false;
                    AuthenticatedUser = "Not authenticated";
                    OperationResult = "Invalid username or password.";
                    OperationResultColor = Brushes.Red;
                }
                 
        }catch (System.Exception ex)
        {
            IsAuthenticated = false;
            AuthenticatedUser = "Not authenticated";
            OperationResult = "An error occurred during login.";
            OperationResultColor = Brushes.Yellow;
            Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error during login: {ex.Message}");
        }
    }

}