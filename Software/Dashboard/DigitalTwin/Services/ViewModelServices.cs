using Microsoft.Extensions.DependencyInjection;
using DigitalTwin.ViewModels;
using DigitalTwin.Database;
using DigitalTwin.Models;
using System;
using DigitalTwin.Data;
using DigitalTwin.Container;
using DigitalTwin.OpcaClient;


namespace DigitalTwin.Services;

public static class ViewModelServiceCollectionExtensions
{
    public static void AddMainViewModelServices(this IServiceCollection collection)
    {
        collection.AddTransient<MainViewModel>();
        collection.AddTransient<LoginViewModel>();
        collection.AddTransient<RegisterViewModel>();
        collection.AddTransient<HomeViewModel>();
        collection.AddSingleton<LaunchViewModel>();
        collection.AddSingleton<ConnectViewModel>();
        collection.AddSingleton<StartViewModel>();
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
        collection.AddSingleton<UserSession>();
        collection.AddSingleton<Func<UserSession>>(provider => () => provider.GetRequiredService<UserSession>());
        collection.AddTransient<HomeInfo>();

    }
    public static void AddProjectPaths(this IServiceCollection collection)
    {
        collection.AddSingleton<ProjectPaths>();
        collection.AddSingleton<Func<ProjectPaths>>(provider => () => provider.GetRequiredService<ProjectPaths>());
        collection.AddSingleton<SystemContainer>();
        collection.AddSingleton<Func<SystemContainer>>(provider => () => provider.GetRequiredService<SystemContainer>());
    }
    public static void AddOpcuaClientServices(this IServiceCollection collection)
    {
        collection.AddSingleton<OpcuaClient>();
        collection.AddSingleton<Func<OpcuaClient>>(provider => () => provider.GetRequiredService<OpcuaClient>());
        collection.AddSingleton<RobotPositionSimulation>();
        collection.AddSingleton<Func<RobotPositionSimulation>>(provider => () => provider.GetRequiredService<RobotPositionSimulation>());
        collection.AddSingleton<ITaskService, TaskService>();
        collection.AddSingleton<Func<ITaskService>>(provider => () => provider.GetRequiredService<ITaskService>());
    }
}