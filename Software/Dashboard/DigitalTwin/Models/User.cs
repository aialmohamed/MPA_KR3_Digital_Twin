
using MongoDB.Bson;
using MongoDB.Bson.Serialization.Attributes;
namespace DigitalTwin.Models;


public class User
{


    [BsonElement("Username")]
    public string Username { get; set; }

    [BsonElement("Password")]
    public string Password { get; set; }
    
    [BsonId]
    [BsonRepresentation(BsonType.ObjectId)]
    public string Id { get; set; }
}
