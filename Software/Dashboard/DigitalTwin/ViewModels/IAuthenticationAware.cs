using System.ComponentModel;
using CommunityToolkit.Mvvm.ComponentModel;

namespace DigitalTwin.ViewModels;


public interface IAuthenticationAware :INotifyPropertyChanged
{

    bool IsAuthenticated { get; }
    string AuthenticatedUser { get; }
}