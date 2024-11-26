

using Opc.Ua;
using Opc.Ua.Client;

namespace OpcuaClient.Client;



public class OpcUaClient
{
    public string Url { get; set; }
    public Session Session { get; private set; }
    private Subscription Subscription { get; set; }

    public OpcUaClient(string url)
    {
        Url = url;
    }

    public void Connect()
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

        config.Validate(ApplicationType.Client).Wait();

        EndpointDescription endpointDescription = CoreClientUtils.SelectEndpoint(Url, useSecurity: false);
        EndpointConfiguration endpointConfiguration = EndpointConfiguration.Create(config);
        ConfiguredEndpoint endpoint = new ConfiguredEndpoint(null, endpointDescription, endpointConfiguration);

        Session = Session.Create(config, endpoint, false, "OpcUaClient", 60000, null, null).Result;
    }

    public void SubscribeToDataChanges(NodeId nodeId, MonitoredItemNotificationEventHandler handler)
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
        Subscription.Create();
    }

    public void Unsubscribe()
    {
        if (Subscription != null)
        {
            Subscription.Delete(true);
            Session.RemoveSubscription(Subscription);
            Subscription = null;
        }
    }

    public void Disconnect()
    {
        Unsubscribe();
        if (Session != null && Session.Connected)
        {
            Session.Close();
            Session.Dispose();
            Session = null;
        }
    }
    public void ReadNodes(ISession session)
    {
        if(session == null || session.Connected == false)
        {
            Console.WriteLine("Session is not connected");
            return;
        }
        try
        {
            ReadValueIdCollection nodesToRead = new ReadValueIdCollection()
            {
                    // Value of ServerStatus
                    new ReadValueId() { NodeId = Variables.Server_ServerStatus, AttributeId = Attributes.Value },
                    // BrowseName of ServerStatus_StartTime
                    new ReadValueId() { NodeId = Variables.Server_ServerStatus_StartTime, AttributeId = Attributes.BrowseName },
                    // Value of ServerStatus_StartTime
                    new ReadValueId() { NodeId = Variables.Server_ServerStatus_StartTime, AttributeId = Attributes.Value }
            };
            Console.WriteLine("Reading nodes...");

            session.Read(
                null,
                0,
                TimestampsToReturn.Both,
                nodesToRead,
                out DataValueCollection results,
                out DiagnosticInfoCollection diagnosticInfos);
                foreach(DataValue result in results)
                {
                    Console.WriteLine($"Node: {result.Value}");
                }
                DataValue namespaceArray = session.ReadValue(Variables.Server_NamespaceArray);
                Console.WriteLine($"NamespaceArray: {namespaceArray.Value}");

        }catch(Exception ex)
        {
            Console.WriteLine($"An error occurred: {ex.Message}");
        }

    }
    public void CallSquareMethod(ISession session)
    {
        if(session == null || session.Connected == false)
        {
            Console.WriteLine("Session is not connected");
            return;
        }
        try
        {
            NodeId objectId = new NodeId("ns=0;i=85");
            NodeId methodId = new NodeId("ns=2;s=Square");
            object[] inputArguments = new object[] { (float)5.0 };
            IList<object> outputArguments = null;

            Console.WriteLine("Calling the Square method...");
            outputArguments = session.Call(
                objectId,
                methodId,
                inputArguments);
            Console.WriteLine($"Result: {outputArguments[0]}");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"An error occurred: {ex.Message}");
        }
    }
    public void BrowsNode(ISession session)
    {
        if(session == null || session.Connected == false)
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

            Console.WriteLine($"Browsing {nodesToBrowse} nodes...");
            ReferenceDescriptionCollection  references = browser.Browse(nodesToBrowse);
            Console.WriteLine("Browse returned {0} results:", references.Count);
            foreach (ReferenceDescription reference in references)
            {
                Console.WriteLine("     DisplayName = {0}, NodeClass = {1}", reference.DisplayName.Text, reference.NodeClass);
            }

        }
        catch (Exception ex)
        {
            Console.WriteLine($"An error occurred: {ex.Message}");
        }
    }
}
