using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Avalonia.Logging;
using DigitalTwin.Data;
using Opc.Ua;
using Opc.Ua.Client;

namespace DigitalTwin.OpcaClient;

public class OpcuaClient
{
    public string Url { get; set; }
    public Session Session { get; private set; }
    private Subscription Subscription { get; set; }

    public OpcuaClient(string url = "opc.tcp://localhost:4840")
    {
        Url = url;
    }

    public bool IsConnected()
    {
        return Session != null && Session.Connected;
    }

    public async Task<bool> ConnectAsync()
    {
        try
        {
            ApplicationConfiguration config = new ApplicationConfiguration()
            {
                ApplicationName = "OpcUaClient",
                ApplicationType = ApplicationType.Client,
                TransportConfigurations = new TransportConfigurationCollection(),
                TransportQuotas = new TransportQuotas { OperationTimeout = 15000 },
                SecurityConfiguration = new SecurityConfiguration
                {
                    ApplicationCertificate = new CertificateIdentifier()
                },
                ClientConfiguration = new ClientConfiguration { DefaultSessionTimeout = 60000 }
            };

            await config.Validate(ApplicationType.Client);

            EndpointDescription endpointDescription = null;

            // Offload endpoint selection to a background thread
            await Task.Run(() =>
            {
                try
                {
                    endpointDescription = CoreClientUtils.SelectEndpoint(Url, useSecurity: false);
                }
                catch (Exception ex)
                {
                    Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error selecting endpoint: {ex.Message}");
                    endpointDescription = null;
                }
            });

            if (endpointDescription == null)
            {
                Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, "Failed to retrieve endpoint description.");
                return false;
            }

            // Continue with endpoint configuration if no exceptions occurred
            EndpointConfiguration endpointConfiguration = EndpointConfiguration.Create(config);
            ConfiguredEndpoint endpoint = new ConfiguredEndpoint(null, endpointDescription, endpointConfiguration);

            // Create the session
            Session = await Session.Create(config, endpoint, false, "OpcUaClient", 60000, null, null);

            if (Session == null)
            {
                Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, "Failed to create OPC UA session.");
                return false;
            }

            return true; // Connection successful
        }
        catch (Exception ex)
        {
            Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error during connection: {ex.Message}");
            return false; // Connection failed
        }
    }

    public async Task UnsubscribeAsync()
    {
        if (Subscription != null)
        {
            await Task.Run(() => Subscription.Delete(true));
            Session.RemoveSubscription(Subscription);
            Subscription = null;
        }
    }

    public async Task DisconnectAsync()
    {
        await UnsubscribeAsync();
        if (Session != null && Session.Connected)
        {
            await Task.Run(() =>
            {
                Session.Close();
                Session.Dispose();
                Session = null;
            });
        }
    }

    public async Task BrowseNodeAsync(ISession session)
    {
        if (session == null || !session.Connected)
        {
            Console.WriteLine("Session is not connected");
            return;
        }

        try
        {
            Browser browser = new Browser()
            {
                Session = session,
                NodeClassMask = (int)NodeClass.Object | (int)NodeClass.Variable | (int)NodeClass.Method,
                ReferenceTypeId = ReferenceTypeIds.HierarchicalReferences,
                IncludeSubtypes = true,
                BrowseDirection = BrowseDirection.Forward,
            };

            NodeId nodesToBrowse = ObjectIds.ObjectsFolder;

            ReferenceDescriptionCollection references = await Task.Run(() => browser.Browse(nodesToBrowse));

            foreach (ReferenceDescription reference in references)
            {
                Logger.TryGet(LogEventLevel.Information, "DigitalTwin")?.Log(
                    this,
                    $"NodeId={reference.NodeId}, BrowseName={reference.BrowseName}, DisplayName={reference.DisplayName}"
                );
            }
        }
        catch (Exception ex)
        {
            Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error during browsing: {ex.Message}");
        }
    }

    public async Task CallSystemMethodAsync(ISession session, string objectIdValue, string methodIdValue)
    {
        if (session == null || !session.Connected)
        {
            Console.WriteLine("Session is not connected");
            return;
        }

        try
        {
            NodeId objectId = new NodeId(objectIdValue);
            NodeId methodId = new NodeId(methodIdValue);

            IList<object> outputArguments = await Task.Run(() => session.Call(objectId, methodId));
        }
        catch (Exception ex)
        {
            Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error during call to method {methodIdValue}: {ex.Message}");
        }
    }

    public async Task SubscribeToVariableAsync(string variableNodeId, Action<DataValue> callback)
    {
        if (Session == null || !Session.Connected)
        {
            Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, "Session is not connected.");
            return;
        }

        try
        {
            if (Subscription == null)
            {
                Subscription = new Subscription(Session.DefaultSubscription)
                {
                    PublishingInterval = 100, // 1-second interval
                    KeepAliveCount = 10,
                    LifetimeCount = 20,
                    Priority = 0,
                };

                Session.AddSubscription(Subscription);
                await Task.Run(() => Subscription.Create());
            }

            MonitoredItem monitoredItem = new MonitoredItem(Subscription.DefaultItem)
            {
                StartNodeId = new NodeId(variableNodeId),
                AttributeId = Attributes.Value,
                SamplingInterval = 1000, // 1-second interval
                QueueSize = 10,
                DiscardOldest = true,
            };

            monitoredItem.Notification += (sender, args) =>
            {
                foreach (var value in monitoredItem.DequeueValues())
                {
                    callback?.Invoke(value);
                }
            };

            Subscription.AddItem(monitoredItem);
            await Task.Run(() => Subscription.ApplyChanges());
        }
        catch (Exception ex)
        {
            Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error subscribing to variable {variableNodeId}: {ex.Message}");
        }
    }

    public async Task CallSendGoalAsync(TaskData taskData)
    {
        if (Session == null || !Session.Connected)
        {
            Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, "Session is not connected.");
            return;
        }

        try
        {

            var objectNodeId = new NodeId("ns=2;s=GoalMethods");
            var methodNodeId = new NodeId("ns=2;s=SendGoal");   

            // Prepare input arguments
            var inputArguments = new List<object>
                {
                    taskData.X,           // x in meters (float)
                    taskData.Y,           // y in meters (float)
                    taskData.Z,           // z in meters (float)
                    taskData.Roll,        // roll in degrees (float)
                    taskData.Pitch,       // pitch in degrees (float)
                    taskData.Yaw,         // yaw in degrees (float)
                    taskData.Gripper      // gripper state (0 or 1, int)
                };


                IList<object> outputArguments = await Task.Run(() =>
                    Session.Call(objectNodeId, methodNodeId, inputArguments.ToArray())
                );

                // Handle the output if necessary
                if (outputArguments != null && outputArguments.Count > 0)
                {
                    var result = outputArguments[0]?.ToString();
                    Logger.TryGet(LogEventLevel.Information, "DigitalTwin")
                        ?.Log(this, $"SendGoal method completed. Result: {result}");
                }
        }
        catch (Exception ex)
        {
            Logger.TryGet(LogEventLevel.Error, "DigitalTwin")
                ?.Log(this, $"Error calling SendGoal method: {ex.Message}");
        }
    }
 
}
