


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
    private RobotPositionReal _robotPositionReal;
    private UserSession _userSession;

    private ITaskService _taskService;
    [ObservableProperty]
    private bool _isSimulationSubscriberChecked;

    [ObservableProperty]
    private bool _isRealSubscriberChecked;

    [ObservableProperty]
    private bool _isMultiTaskChecked;

    [ObservableProperty]
    private bool _isDigitalTwinChecked;

#region Task Properties
    [ObservableProperty]
    private string _taskStatus;

#endregion

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
    public StartViewModel(Func<OpcuaClient> getOpcuaClient,Func<RobotPositionSimulation> getRobotPositionSimulation,Func<RobotPositionReal> getRobotPositionReal , Func<ITaskService> getTaskService ,Func<UserSession> getUserSession)
    {
        PageNames = ApplicationPageNames.Start;
        _robotPositionSimulation = getRobotPositionSimulation();
        _robotPositionReal = getRobotPositionReal();
        _opcaClient = getOpcuaClient();
        _robotPositionSimulation.DataUpdated += OnSimulationDataUpdated;
        _robotPositionSimulation.MovementStatusChanged += OnMovementStatusChanged;
        _robotPositionReal.DataUpdated += OnRealDataUpdated;
        //_robotPositionReal.MovementStatusChanged += OnMovementStatusChanged;
        _taskService = getTaskService();
        _userSession = getUserSession();
    }

    [RelayCommand]
    public async Task StartDigitalTwin()
    {
        if(IsDigitalTwinChecked)
        {
            await _opcaClient.CallSystemMethodAsync(_opcaClient.Session,"ns=2;s=DigitalTwinMethods","ns=2;s=LaunchDigitalTwin");
        }else
        {
            await _opcaClient.CallSystemMethodAsync(_opcaClient.Session,"ns=2;s=DigitalTwinMethods","ns=2;s=ShutdownDigitalTwin");
        }
    }

    [RelayCommand]
    public async Task StartRqt()
    {
        
        Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, "Starting Rqt...");
        await _opcaClient.CallSystemMethodAsync(_opcaClient.Session,"ns=2;s=RqtMethods","ns=2;s=LaunchRqt");
        Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, "Rqt started.");
    }

    [RelayCommand]
    public async Task SubScribeToSimulationPosition()
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
    public async Task SubScribeToRealPosition()
    {
        if(IsRealSubscriberChecked)
        {
            await _opcaClient.CallSystemMethodAsync(_opcaClient.Session,"ns=2;s=CartesianPoseMethods","ns=2;s=GetRealCartesianPose");
            await _robotPositionReal.StartDataSubscriptionAsync();
        }
        else
        {
            await _opcaClient.CallSystemMethodAsync(_opcaClient.Session,"ns=2;s=CartesianPoseMethods","ns=2;s=ShutdownRealCartesianPose");
        }
    }

    [RelayCommand]
    public void StartSystem()
    {
        IsSimulationSubscriberChecked = true;
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
        TaskStatus = $"Adding Simple Task: {task.ToString()}";
        await _taskService.AddTaskAsync(_userSession.Username,task,TaskDataStatus.Idle);       
        Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, $"Task added successfully.");
        TaskStatus = $"Task Added: {task.ToString()}";
        
    }

    [RelayCommand]
    public async Task StartTask()
    {
        async Task ExecuteTask(TaskData task, string taskDescription)
        {
            try
            {
                Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, $"Sending task: {taskDescription}");
                TaskStatus = $"Task Started: {taskDescription}";
                await _opcaClient.CallSendGoalAsync(task);
                TaskStatus = $"Task Sent: {taskDescription}";
                await WaitForRobotToStopAsync(5000); // Timeout after 5 seconds
                TaskStatus = $"Task Completed: {taskDescription}";
            }
            catch (Exception ex)
            {
                Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error executing task: {taskDescription}. Exception: {ex.Message}");
                TaskStatus = $"Task Failed: {taskDescription}";
            }
        }

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

            await ExecuteTask(task, task.ToString());
        }
        else
        {
            // Home Task
            var builder = new TaskBuilder()
                .SetX(0.35)
                .SetY(0.0)
                .SetZ(0.6)
                .SetRoll(0.0)
                .SetPitch(0.0)
                .SetYaw(0.0)
                .SetGripper(false);
            var homeTask = builder.Build();

            await ExecuteTask(homeTask, "Home Task");
            await Task.Delay(2500); // Delay between tasks

            // Multi-Task
            var tasks = await _taskService.GetTasksByUsernameAsync(_userSession.Username);
            foreach (var task in tasks)
            {
                await ExecuteTask(task.TaskData, task.TaskData.ToString());
                await Task.Delay(2500); // Delay between tasks
            }

            TaskStatus = "All tasks completed.";
        }
    }

    [RelayCommand]
    public async Task ResetTasks()
    {
        TaskStatus = "Deleting all tasks...";
        await _taskService.DeleteAllTasksAsync();
        TaskStatus = "All tasks deleted.";
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
    private void OnRealDataUpdated(string propertyName, string newValue)
    {
              // Update the corresponding ViewModel property
        switch (propertyName)
        {
            case nameof(RobotPositionReal.XCoordinate):
                XCoords_real = newValue;
                break;
            case nameof(RobotPositionReal.YCoordinate):
                YCoords_real = newValue;
                break;
            case nameof(RobotPositionReal.ZCoordinate):
                ZCoords_real = newValue;
                break;
            case nameof(RobotPositionReal.Roll):
                Roll_real = newValue;
                break;
            case nameof(RobotPositionReal.Pitch):
                Pitch_real = newValue;
                break;
            case nameof(RobotPositionReal.Yaw):
                Yaw_real = newValue;
                break;
            case nameof(RobotPositionReal.Gripper):
                Gripper_real = newValue;
                break;
        }

    }

    private async Task WaitForRobotToStopAsync(int timeoutMs = 5000)
    {

            if (_movementCompletionSource != null && !_movementCompletionSource.Task.IsCompleted)
            {
                throw new InvalidOperationException("Another task is already waiting for robot to stop.");
            }

            _movementCompletionSource = new TaskCompletionSource<bool>();
            _robotPositionSimulation.MovementStatusChanged += OnRobotStopped;

        var timeoutTask = Task.Delay(timeoutMs).ContinueWith(_ => false);

        var completedTask = await Task.WhenAny(_movementCompletionSource.Task, timeoutTask);

        if (completedTask == timeoutTask)
        {
            Logger.TryGet(LogEventLevel.Warning, "DigitalTwin")?.Log(this, "Timeout reached. Assuming robot has stopped.");
            _movementCompletionSource.TrySetResult(true);
        }

        _robotPositionSimulation.MovementStatusChanged -= OnRobotStopped;

        await _movementCompletionSource.Task;
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
        if (!isMoving)
        {
            _movementCompletionSource?.TrySetResult(true);
            _robotPositionSimulation.MovementStatusChanged -= OnRobotStopped; // Ensure cleanup
        }
    }

}   