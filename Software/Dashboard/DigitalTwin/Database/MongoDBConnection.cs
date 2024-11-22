
using MongoDB.Driver;
using MongoDB.Bson;
using Microsoft.VisualBasic;
using MongoDB.Driver.Core.Configuration;


namespace DigitalTwin.Database;


public class MongoDBConnection
{
    private readonly MongoClient _client;

    public MongoDBConnection(string connectionString = "mongodb://localhost:27017")
    {
        var settings = MongoClientSettings.FromConnectionString(connectionString);
        settings.ServerApi = new ServerApi(ServerApiVersion.V1);

        _client = new MongoClient(settings);
    }

    public MongoClient GetClient()
    {
        return _client;
    }
}
