
using System;
using System.Threading.Tasks;
using Avalonia.Media;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using DigitalTwin.Data;
using DigitalTwin.Models;
using DigitalTwin.Services;

namespace DigitalTwin.ViewModels;



public partial class RegisterViewModel : PageViewModel
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
    
    public RegisterViewModel(IAuthenticationService authenticationService)
    {
        _authenticationService = authenticationService;
        PageNames = ApplicationPageNames.Register;
    }

    [RelayCommand]
    private async Task Register()
    {
        try
        {
            // save user to database
            if (string.IsNullOrEmpty(Username) || string.IsNullOrEmpty(Password))
            {
                OperationResult = "User name and password are required.";
                OperationResultColor = Brushes.Red;
                return;

            }
            await _authenticationService.RegisterUserAsync(Username, Password);
            OperationResult = "User registered successfully!";
            OperationResultColor = Brushes.Green;

        }catch(Exception ex)
        {
            OperationResult = ex.Message;
            OperationResultColor = Brushes.Red;
        }
        
    }
}