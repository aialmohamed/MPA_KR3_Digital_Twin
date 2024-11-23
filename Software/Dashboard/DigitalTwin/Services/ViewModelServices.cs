using Microsoft.Extensions.DependencyInjection;
using DigitalTwin.ViewModels;
using DigitalTwin.Database;


namespace DigitalTwin.Services;

public static class ViewModelServiceCollectionExtensions
{
    public static void AddMainViewModelServices(this IServiceCollection collection)
    {
        collection.AddTransient<MainViewModel>();
        collection.AddTransient<LoginViewModel>();
        collection.AddTransient<RegisterViewModel>();
    }
    public static void AddDataBaseServices(this IServiceCollection collection)
    {
        collection.AddSingleton<MongoDBConnection>(sp =>
        {
            var connectionString = "mongodb://localhost:27017";
            return new MongoDBConnection(connectionString);
        });
        collection.AddTransient<UserService>();
    }

    public static void AddAuthenticationServices(this IServiceCollection collection)
    {

        collection.AddSingleton<IAuthenticationService, AuthenticationService>();
    }
}