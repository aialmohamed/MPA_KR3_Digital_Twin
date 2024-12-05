using System;
using System.Linq;
using System.Threading.Tasks;
using Avalonia.Logging;
using Opc.Ua;
using DigitalTwin.OpcaClient;

namespace DigitalTwin.Models
{
    public class RobotPositionSimulation
    {
        private readonly OpcuaClient _opcUaClient;

        public event Action<string, string> DataUpdated;
        public event Action<bool> MovementStatusChanged; // Event to indicate movement state (true = moving, false = stationary)

        private readonly double _movementThreshold = 5e-3; // Movement tolerance for X, Y, Z, Roll, Pitch, Yaw
        private readonly int _movementCheckIntervalMs = 800; // Check interval in milliseconds

        private string _xCoordinate;
        public string XCoordinate => _xCoordinate;

        private string _yCoordinate;
        public string YCoordinate => _yCoordinate;

        private string _zCoordinate;
        public string ZCoordinate => _zCoordinate;

        private string _roll;
        public string Roll => _roll;

        private string _pitch;
        public string Pitch => _pitch;

        private string _yaw;
        public string Yaw => _yaw;

        private string _gripper;
        public string Gripper => _gripper;

        private double[] _lastState = new double[6]; // Last known state: [X, Y, Z, Roll, Pitch, Yaw]
        private bool _lastGripperState; // Last known gripper state
        private bool _isMoving;

        public RobotPositionSimulation(OpcuaClient opcUaClient)
        {
            _opcUaClient = opcUaClient ?? throw new ArgumentNullException(nameof(opcUaClient));
        }

        public async Task StartDataSubscriptionAsync()
        {
            if (_opcUaClient.Session == null || !_opcUaClient.IsConnected())
            {
                Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, "OPC UA session is not connected. Cannot start data subscription.");
                return;
            }

            try
            {
                Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, "Starting data subscription for simulation position.");

                await SubscribeToCoordinateAsync("ns=2;i=45", value => UpdateProperty(nameof(XCoordinate), ref _xCoordinate, value));
                await SubscribeToCoordinateAsync("ns=2;i=46", value => UpdateProperty(nameof(YCoordinate), ref _yCoordinate, value));
                await SubscribeToCoordinateAsync("ns=2;i=47", value => UpdateProperty(nameof(ZCoordinate), ref _zCoordinate, value));
                await SubscribeToCoordinateAsync("ns=2;i=48", value => UpdateProperty(nameof(Roll), ref _roll, value));
                await SubscribeToCoordinateAsync("ns=2;i=49", value => UpdateProperty(nameof(Pitch), ref _pitch, value));
                await SubscribeToCoordinateAsync("ns=2;i=50", value => UpdateProperty(nameof(Yaw), ref _yaw, value));
                await SubscribeToCoordinateAsync("ns=2;i=51", value => UpdateProperty(nameof(Gripper), ref _gripper, value));

                StartMonitoringMovement();

                Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, "Data subscription for simulation position started successfully.");
            }
            catch (Exception ex)
            {
                Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error during subscription: {ex.Message}");
            }
        }

        private void StartMonitoringMovement()
        {
            Task.Run(async () =>
            {
                while (true)
                {
                    await Task.Delay(_movementCheckIntervalMs);

                    if (TryParseState(out var currentState, out var gripperState))
                    {
                        var isMoving = IsMoving(_lastState, _lastGripperState, currentState, gripperState);

                        if (isMoving != _isMoving)
                        {
                            _isMoving = isMoving;
                            MovementStatusChanged?.Invoke(_isMoving); // Notify about movement state
                        }

                        // Update last known state
                        _lastState = currentState;
                        _lastGripperState = gripperState;
                    }
                }
            });
        }

        private bool IsMoving(double[] lastState, bool lastGripper, double[] currentState, bool currentGripper)
        {
            // Check positional movement (X, Y, Z, Roll, Pitch, Yaw)
            for (int i = 0; i < lastState.Length; i++)
            {
                if (Math.Abs(currentState[i] - lastState[i]) > _movementThreshold)
                {
                    return true; // Movement detected in one of the parameters
                }
            }

            // Check gripper state change
            if (lastGripper != currentGripper)
            {
                return true; // Gripper state changed
            }

            return false; // No movement
        }

        private bool TryParseState(out double[] state, out bool gripperState)
        {
            state = new double[6];
            gripperState = false;

            if (double.TryParse(XCoordinate, out var x) &&
                double.TryParse(YCoordinate, out var y) &&
                double.TryParse(ZCoordinate, out var z) &&
                double.TryParse(Roll, out var roll) &&
                double.TryParse(Pitch, out var pitch) &&
                double.TryParse(Yaw, out var yaw) &&
                bool.TryParse(Gripper, out var gripper))
            {
                state[0] = x;
                state[1] = y;
                state[2] = z;
                state[3] = roll;
                state[4] = pitch;
                state[5] = yaw;
                gripperState = gripper;
                return true;
            }

            Logger.TryGet(LogEventLevel.Warning, "DigitalTwin")?.Log(this, "Failed to parse robot state values.");
            return false;
        }

        private async Task SubscribeToCoordinateAsync(string nodeId, Action<string> onDataReceived)
        {
            try
            {
                await _opcUaClient.SubscribeToVariableAsync(nodeId, dataValue =>
                {
                    var formattedValue = ProcessDataValue(nodeId, dataValue);
                    onDataReceived?.Invoke(formattedValue);
                });

                Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(this, $"Successfully subscribed to variable: {nodeId}");
            }
            catch (Exception ex)
            {
                Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error subscribing to variable {nodeId}: {ex.Message}");
            }
        }

        private void UpdateProperty(string propertyName, ref string propertyField, string newValue)
        {
            if (propertyField != newValue)
            {
                propertyField = newValue;
                DataUpdated?.Invoke(propertyName, newValue); // Notify listeners
            }
        }

        private string ProcessDataValue(string nodeId, DataValue dataValue)
        {
            if (dataValue?.Value == null)
            {
                Logger.TryGet(LogEventLevel.Warning, "DigitalTwin")?.Log(this, "Received null data value.");
                return "Invalid Data";
            }

            if (nodeId == "ns=2;i=51") // NodeId for Gripper (boolean)
            {
                if (bool.TryParse(dataValue.Value.ToString(), out bool boolValue))
                {
                    return boolValue.ToString(); // Converts true/false to "true"/"false"
                }
                Logger.TryGet(LogEventLevel.Warning, "DigitalTwin")?.Log(this, "Invalid boolean value received.");
                return "Invalid Data";
            }

            // For all other coordinates (float values)
            if (float.TryParse(dataValue.Value.ToString(), out float floatValue))
            {
                return floatValue.ToString("F2"); // Format float to 2 decimal places
            }

            Logger.TryGet(LogEventLevel.Warning, "DigitalTwin")?.Log(this, "Invalid numeric value received.");
            return "Invalid Data";
        }
    }
}
