using Avalonia;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Data.Core;
using Avalonia.Data.Core.Plugins;
using Avalonia.Markup.Xaml;
using DigitalTwin.Services;
using DigitalTwin.ViewModels;
using DigitalTwin.Views;
using Microsoft.Extensions.DependencyInjection;

namespace DigitalTwin;

public partial class App : Application
{
        public override void Initialize()
    {
        AvaloniaXamlLoader.Load(this);
    }

    public override void OnFrameworkInitializationCompleted()
    {
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
        {
            // Line below is needed to remove Avalonia data validation.
            // Without this line you will get duplicate validations from both Avalonia and CT
            
            BindingPlugins.DataValidators.RemoveAt(0);

            var collection = new ServiceCollection();
            collection.AddDataBaseServices();
            collection.AddAuthenticationServices();
            collection.AddMainViewModelServices();


            var provider = collection.BuildServiceProvider();
            var vm = provider.GetRequiredService<MainViewModel>();
            desktop.MainWindow = new MainView
            {
                DataContext = vm
            };
        }

        base.OnFrameworkInitializationCompleted();
    }
}