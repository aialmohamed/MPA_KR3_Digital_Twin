
using System.Threading.Tasks;
using Avalonia.Logging;
using Avalonia.Media;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using DigitalTwin.Data;
using DigitalTwin.Models;
using DigitalTwin.Services;

namespace DigitalTwin.ViewModels;

public partial class LoginViewModel : PageViewModel,IAuthenticationAware
{
    private readonly IAuthenticationService _authenticationService;
    private readonly UserSession _userSession;

    [ObservableProperty]
    private string _username = string.Empty;

    [ObservableProperty]
    private string _password = string.Empty;

    [ObservableProperty]
    private string _operationResult = string.Empty;

    [ObservableProperty]
    private IBrush _operationResultColor = Brushes.Black;


    //deaigner 

    public LoginViewModel()
    {
    }
    public LoginViewModel(IAuthenticationService authenticationService,UserSession userSession)
    {
         PageNames = ApplicationPageNames.Login;
         
        _authenticationService = authenticationService;
        _userSession = userSession;
    }

    [RelayCommand]
    private async Task Login()
    {
        // we should creat a usersession here and let it authenticate the user
        try
        {
                var isAuthenticated  = await _authenticationService.AuthenticateUserAsync(Username, Password);
                if (isAuthenticated)
                {
                    // Update the shared UserSession
                    _userSession.Username = Username;
                    _userSession.IsAuthenticated = true;
                    OperationResult = "Login successful!";
                    OperationResultColor = Brushes.Green;
                }
                else
                {

                    _userSession.ClearSession(); 
                    OperationResult = "Invalid username or password.";
                    OperationResultColor = Brushes.Red;
                }
                 
        }catch (System.Exception ex)
        {
            _userSession.ClearSession(); 
            OperationResult = "An error occurred during login.";
            OperationResultColor = Brushes.Yellow;
            Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error during login: {ex.Message}");
        }
    }

}