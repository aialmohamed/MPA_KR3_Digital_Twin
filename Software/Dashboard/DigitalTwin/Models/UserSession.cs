


using CommunityToolkit.Mvvm.ComponentModel;

namespace DigitalTwin.Models;

public partial class UserSession : ObservableObject
{    
    [ObservableProperty]
    private string _username = string.Empty;

    [ObservableProperty]
    private bool _isAuthenticated;

    public void ClearSession()
    {
        Username = string.Empty;
        IsAuthenticated = false;
    }
}