


using System;
using System.Threading.Tasks;
using Avalonia.Media;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using DigitalTwin.Container;
using DigitalTwin.Data;

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
                ContainerStatus = "Container is running , waiting for the OPC UA server ...";
                ContainerStatusColor = Brushes.Yellow;
                bool isServerReady = await _systemContainer.WaitForOpcUaServerAsync("opc.tcp://localhost:4840");

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
            }
            catch (Exception ex)
            {
                Console.WriteLine($"An error occurred: {ex.Message}");
            }
        });
    }
    [RelayCommand]
    private void BuildImage()
    {
                Task.Run(async () =>
        {
            try
            {
                string imageName = "ibo311/kr3r540_digital_twin:v1.0";
                string dockerFilePath = _systemContainer.DockerFilePath; 
                ImageStatusColor = Brushes.Yellow;
                ImageStatusText = "Building Docker Image ...";
                IsContainerRunning = false;
                await _systemContainer.EnsureDockerImageExistsAsync(imageName, dockerFilePath);
                ImageStatusColor = Brushes.Green;
                ImageStatusText = "Docker Image Done Building , you can run the container now";
                IsContainerRunning = true;

            }
            catch (Exception ex)
            {
                Console.WriteLine($"An error occurred: {ex.Message}");
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