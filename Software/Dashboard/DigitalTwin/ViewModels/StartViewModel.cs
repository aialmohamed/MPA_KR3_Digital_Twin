


using System;
using CommunityToolkit.Mvvm.ComponentModel;
using DigitalTwin.Data;
using DigitalTwin.OpcaClient;
namespace DigitalTwin.ViewModels;

using System.ComponentModel;
using System.Threading.Tasks;
using Avalonia.Logging;
using CommunityToolkit.Mvvm.Input;
using DigitalTwin.Factories;
using DigitalTwin.Models;
using DigitalTwin.Services;

public partial class StartViewModel : PageViewModel
    {
    private TaskCompletionSource<bool> _movementCompletionSource;
    private bool _isRobotMoving = true;
    private RobotPositionSimulation _robotPositionSimulation;
    private UserSession _userSession;

    private ITaskService _taskService;
    [ObservableProperty]
    private bool _isSimulationSubscriberChecked = true;

    [ObservableProperty]
    private bool _isMultiTaskChecked;

#region cards Properties
    [ObservableProperty]
    private string _xCoords_real;
    [ObservableProperty]
    private string _yCoords_real;
    [ObservableProperty]
    private string _zCoords_real;
    [ObservableProperty]
    private string _roll_real;
    [ObservableProperty]
    private string _pitch_real;
    [ObservableProperty]
    private string _yaw_real;
    [ObservableProperty]
    private string _gripper_real;

    [ObservableProperty]
    private string _xCoords_sim;
    [ObservableProperty]
    private string _yCoords_sim;
    [ObservableProperty]
    private string _zCoords_sim;
    [ObservableProperty]
    private string _roll_sim;
    [ObservableProperty]
    private string _pitch_sim;
    [ObservableProperty]
    private string _yaw_sim;
    [ObservableProperty]
    private string _gripper_sim;
#endregion

#region Sliders Properties
    [ObservableProperty]
    private double _sliderX;
    [ObservableProperty]
    private double _sliderY;
    [ObservableProperty]
    private double _sliderZ;
    [ObservableProperty]
    private double _sliderRoll;
    [ObservableProperty]
    private double _sliderPitch;
    [ObservableProperty]
    private double _sliderYaw;
    [ObservableProperty]
    private bool _checkBoxGripper;


#endregion
    private readonly OpcuaClient _opcaClient;
    public StartViewModel(Func<OpcuaClient> getOpcuaClient,Func<RobotPositionSimulation> getRobotPositionSimulation, Func<ITaskService> getTaskService ,Func<UserSession> getUserSession)
    {
        PageNames = ApplicationPageNames.Start;
        _robotPositionSimulation = getRobotPositionSimulation();
        _opcaClient = getOpcuaClient();
        _robotPositionSimulation.DataUpdated += OnSimulationDataUpdated;
        _robotPositionSimulation.MovementStatusChanged += OnMovementStatusChanged;

        _taskService = getTaskService();
        _userSession = getUserSession();
    }

    [RelayCommand]
    public async Task SubScribeToSimulationPositionAsync()
    {
        if(IsSimulationSubscriberChecked)
        {         
            await _opcaClient.CallSystemMethodAsync(_opcaClient.Session,"ns=2;s=CartesianPoseMethods","ns=2;s=GetSimCartesianPose");
            await _robotPositionSimulation.StartDataSubscriptionAsync();
        }
        else
        {
            await _opcaClient.CallSystemMethodAsync(_opcaClient.Session,"ns=2;s=CartesianPoseMethods","ns=2;s=ShutdownSimCartesianPose");
        }

    }

    [RelayCommand]
    public void StartSystem()
    {
        Task.Run(async() =>
        {
            await _opcaClient.CallSystemMethodAsync(_opcaClient.Session,"ns=2;s=ControlMethods","ns=2;s=LaunchRos2Simulation");
        });
        
    }

    [RelayCommand]
    public void StopSystem()
    {
        Task.Run(async() =>
        {
            await _opcaClient.CallSystemMethodAsync(_opcaClient.Session,"ns=2;s=ControlMethods","ns=2;s=KillRos2Simulation");
        });
    }
    [RelayCommand]
    public async Task SetTask()
    {
        // build Data : 
        var  builder = new TaskBuilder()
        .SetX(SliderX)
        .SetY(SliderY)
        .SetZ(SliderZ)
        .SetRoll(SliderRoll)
        .SetPitch(SliderPitch)
        .SetYaw(SliderYaw)
        .SetGripper(CheckBoxGripper);
        var task = builder.Build();
        Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, $"Task to be added: {task.ToString()}");
        await _taskService.AddTaskAsync(_userSession.Username,task,TaskDataStatus.Idle);       
        Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, $"Task added successfully.");
        
    }

    [RelayCommand]
    public async Task StartTask()
    {
        if (!IsMultiTaskChecked)
        {
            // Single Task
            var builder = new TaskBuilder()
                .SetX(SliderX)
                .SetY(SliderY)
                .SetZ(SliderZ)
                .SetRoll(SliderRoll)
                .SetPitch(SliderPitch)
                .SetYaw(SliderYaw)
                .SetGripper(CheckBoxGripper);
            var task = builder.Build();
            Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, $"Sending task: {task.ToString()}");
            await _opcaClient.CallSendGoalAsync(task);
            Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, $"Task sent successfully.");
            await WaitForRobotToStopAsync();
            Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, "Robot has stopped.");
        }
        else
        {
            // Start at Home Position :
            var builder = new TaskBuilder()
                .SetX(0.35)
                .SetY(0.0)
                .SetZ(0.6)
                .SetRoll(0.0)
                .SetPitch(0.0)
                .SetYaw(0.0)
                .SetGripper(false);
            var HomeTask = builder.Build();
            await _opcaClient.CallSendGoalAsync(HomeTask);

            // Multi-Task
            var tasks = await _taskService.GetTasksByUsernameAsync(_userSession.Username);
            foreach (var task in tasks)
            {
                Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, $"Sending task: {task.TaskData.ToString()}");
                await _opcaClient.CallSendGoalAsync(task.TaskData);
                Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, $"Task sent successfully.");
                await WaitForRobotToStopAsync();
                Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, "Robot has stopped.");
            }
        }
    }

    [RelayCommand]
    public async Task ResetTasks()
    {
        Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, $"Resetting all tasks.");
        await _taskService.DeleteAllTasksAsync();
        Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, $"All tasks deleted successfully.");
    }

    private void OnSimulationDataUpdated(string propertyName, string newValue)
    {
        // Update the corresponding ViewModel property
        switch (propertyName)
        {
            case nameof(RobotPositionSimulation.XCoordinate):
                XCoords_sim = newValue;
                break;
            case nameof(RobotPositionSimulation.YCoordinate):
                YCoords_sim = newValue;
                break;
            case nameof(RobotPositionSimulation.ZCoordinate):
                ZCoords_sim = newValue;
                break;
            case nameof(RobotPositionSimulation.Roll):
                Roll_sim = newValue;
                break;
            case nameof(RobotPositionSimulation.Pitch):
                Pitch_sim = newValue;
                break;
            case nameof(RobotPositionSimulation.Yaw):
                Yaw_sim = newValue;
                break;
            case nameof(RobotPositionSimulation.Gripper):
                Gripper_sim = newValue;
                break;
        }
    }

    private Task WaitForRobotToStopAsync()
    {
        _movementCompletionSource = new TaskCompletionSource<bool>();

        // Set up a one-time event handler
        _robotPositionSimulation.MovementStatusChanged += OnRobotStopped;

        return _movementCompletionSource.Task;
    }
    private void OnMovementStatusChanged(bool isMoving)
    {
        _isRobotMoving = isMoving;

        if (!isMoving && _movementCompletionSource != null)
        {
            _movementCompletionSource.TrySetResult(true); // Resolve waiting task
            _movementCompletionSource = null;

            Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, "Robot has stopped.");
        }
        else if (isMoving)
        {
            Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, "Robot is moving.");
        }
    }
    private void OnRobotStopped(bool isMoving)
    {
        if (!isMoving) // Robot has stopped
        {
            _movementCompletionSource?.TrySetResult(true);

            // Clean up the event handler
            _robotPositionSimulation.MovementStatusChanged -= OnRobotStopped;
        }
    }
}   