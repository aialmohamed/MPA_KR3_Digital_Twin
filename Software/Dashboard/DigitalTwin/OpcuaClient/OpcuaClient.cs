using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Avalonia.Logging;
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

public async Task ConnectAsync()
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

        // Continue with endpoint configuration if no exceptions occurred
        EndpointConfiguration endpointConfiguration = EndpointConfiguration.Create(config);
        ConfiguredEndpoint endpoint = new ConfiguredEndpoint(null, endpointDescription, endpointConfiguration);

        // Create the session
        Session = await Session.Create(config, endpoint, false, "OpcUaClient", 60000, null, null);
    }
    catch (Exception ex)
    {
        Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error during connection: {ex.Message}");
    }
}

    public async Task SubscribeToDataChangesAsync(NodeId nodeId, MonitoredItemNotificationEventHandler handler)
    {
        // Create a subscription with a publishing interval of 1000 ms
        Subscription = new Subscription(Session.DefaultSubscription) { PublishingInterval = 1000 };

        // Create a monitored item
        MonitoredItem monitoredItem = new MonitoredItem(Subscription.DefaultItem)
        {
            DisplayName = nodeId.ToString(),
            StartNodeId = nodeId
        };

        // Add the handler
        monitoredItem.Notification += handler;

        // Add the item to the subscription
        Subscription.AddItem(monitoredItem);

        // Add the subscription to the session
        Session.AddSubscription(Subscription);

        // Create the subscription on the server
        await Task.Run(() => Subscription.Create());
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

}