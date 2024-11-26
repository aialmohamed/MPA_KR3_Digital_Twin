using Opc.Ua;
using Opc.Ua.Client;
using Opc.Ua.Configuration;
using OpcuaClient.Client;
using System;
using System.Threading.Tasks;

class Program
{
    static void OnDataChange(MonitoredItem item, MonitoredItemNotificationEventArgs e)
    {
        foreach (var value in item.DequeueValues())
        {
            Console.WriteLine($"New value for {item.DisplayName}: {value.Value}");
        }
    }
    static async Task Main(string[] args)
    {
        Console.WriteLine("Client Demo");
        string url = "opc.tcp://localhost:4840";
        var client = new OpcUaClient(url);
        try
        {
            // Connect to the server
            client.Connect();
            
            Console.WriteLine("Connected to the server.");

            // Subscribe to a node (replace with a valid node ID from your server
            // listing nodes : 
            client.CallSquareMethod(client.Session);
            client.BrowsNode(client.Session);


            Console.WriteLine("Press any key to exit...");
            Console.ReadKey();


            client.Disconnect();
            Console.WriteLine("Disconnected from the server.");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"An error occurred: {ex.Message}");
        }
    }
}
