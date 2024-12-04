


using System;
using DigitalTwin.Data;
using MongoDB.Bson;
using MongoDB.Bson.Serialization.Attributes;

namespace DigitalTwin.Models;

    public enum TaskDataStatus
    {
        Idle,
        Doing,
        Done
    }
    public class TaskDataModel
    {
        [BsonId]
        [BsonRepresentation(BsonType.ObjectId)]
        public string Id { get; set; }

        [BsonElement("Username")]
        public string Username { get; set; }

        [BsonElement("TaskData")]
        public TaskData TaskData { get; set; } 

        [BsonElement("Status")]
        public TaskDataStatus Status { get; set; } 
        
        [BsonElement("CreatedAt")]
        public DateTime CreatedAt { get; set; } // Timestamp for FIFO ordering
    }