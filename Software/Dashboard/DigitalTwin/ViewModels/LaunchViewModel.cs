


using System;
using System.Threading.Tasks;
using Avalonia.Media;
using Avalonia.Threading;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using DigitalTwin.Container;
using DigitalTwin.Data;
using static System.Net.Mime.MediaTypeNames;

namespace DigitalTwin.ViewModels;

// Starting the Docker Container and check status of docker 
public partial class LaunchViewModel : PageViewModel
{
    private readonly SystemContainer _systemContainer;
    [ObservableProperty]
    private bool _doseImageExist = false;

    [ObservableProperty]
    private IBrush _imageStatusColor;
    [ObservableProperty]
    private string _imageStatusText;

    [ObservableProperty]
    private string _containerStatus = "Container is not running";
    [ObservableProperty]
    private IBrush _containerStatusColor = Brushes.Red;

    [ObservableProperty]
    private bool _isContainerRunning = false;
    public LaunchViewModel(Func<SystemContainer> getSystemContainer )
    {
        PageNames = ApplicationPageNames.Launch;
        _systemContainer = getSystemContainer();
        Task.Run(async () =>
        {
            try
            {
                DoseImageExist = await _systemContainer.CheckDockerImageExists("ibo311/kr3r540_digital_twin:v1.0");
                if(DoseImageExist)
                {
                    ImageStatusColor = Brushes.Green;
                    ImageStatusText = "Docker Image Exists";
                }else
                {
                    ImageStatusColor = Brushes.Red;
                    ImageStatusText = "Docker Image does not exist , Click the Build Image Button to build it";
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"An error occurred: {ex.Message}");
            }
        });

    }


    [RelayCommand]
    private void StartContainer()
    {
    
    Task.Run(async () =>
        {
            try
            {
                await _systemContainer.LaunchDockerContainerInBackgroundAsync();

                 await Dispatcher.UIThread.InvokeAsync(() =>
                {
                        ContainerStatus = "Container is running , waiting for the OPC UA server ...";
                        ContainerStatusColor = Brushes.Yellow;
                });

                bool isServerReady = await _systemContainer.WaitForOpcUaServerAsync("opc.tcp://localhost:4840");
                await Dispatcher.UIThread.InvokeAsync(() =>
                {                
                    if (isServerReady)
                    {
                        IsContainerRunning = true;
                        ContainerStatus = "The OPC UA server is ready.";
                        ContainerStatusColor = Brushes.Green;
                        
                    }
                    else
                    {
                        ContainerStatus = "The server failed to start within the timeout period.";
                        ContainerStatusColor = Brushes.Red;
                    } 
                });

            }
            catch (Exception ex)
            {
                Console.WriteLine($"An error occurred: {ex.Message}");
            }
        });
    }
    [RelayCommand]private void BuildImage()
{
    Task.Run(async () =>
    {
        try
        {
            string imageName = "ibo311/kr3r540_digital_twin:v1.0";
            string dockerFilePath = _systemContainer.RootPath; 
            await Dispatcher.UIThread.InvokeAsync(() =>
            {
                ImageStatusColor = Brushes.Yellow;
                ImageStatusText = "Building Docker Image ...";
            });
            bool image_built = await _systemContainer.EnsureDockerImageExistsAsync(imageName, dockerFilePath);
            await Dispatcher.UIThread.InvokeAsync(() =>
            {
                if (!image_built)
                {
                    ImageStatusColor = Brushes.Red;
                    ImageStatusText = "Failed to build Docker Image";
                    DoseImageExist = false;
                }
                else
                {
                    ImageStatusColor = Brushes.Green;
                    ImageStatusText = "Docker Image Done Building, you can run the container now";
                    DoseImageExist = true;

                }
            });
        }
        catch (Exception ex)
        {
            // Handle exceptions and update the UI
             await Dispatcher.UIThread.InvokeAsync(() =>
            {
                ImageStatusColor = Brushes.Red;
                ImageStatusText = $"An error occurred: {ex.Message}";
            });
        }
    });
}

    [RelayCommand]
    private void StopContainer()
    {
        Task.Run(async () =>
        {
            try
            {
                ContainerStatus = "Stopping Container ...";
                ContainerStatusColor = Brushes.Yellow;
                await _systemContainer.StopAndRemoveDockerContainerAsync("kr3r540_digital_twin");
                ContainerStatus = "Container is stopped";
                ContainerStatusColor = Brushes.Red;
                IsContainerRunning = false;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"An error occurred: {ex.Message}");
            }
        });

    }

}