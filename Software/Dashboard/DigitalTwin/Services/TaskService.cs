using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using DigitalTwin.Data;
using DigitalTwin.Database;
using DigitalTwin.Models;
using MongoDB.Driver;

namespace DigitalTwin.Services
{
    public class TaskService : ITaskService
    {
        private readonly IMongoCollection<TaskDataModel> _taskCollection;

        public TaskService(MongoDBConnection mongoConnection, string databaseName = "DigitalTwinDB")
        {
            var database = mongoConnection.GetClient().GetDatabase(databaseName);
            _taskCollection = database.GetCollection<TaskDataModel>("tasks");
        }

        public async Task AddTaskAsync(string username, TaskData taskData, TaskDataStatus status = TaskDataStatus.Idle)
        {
            var newTask = new TaskDataModel
            {
                Username  = username,
                TaskData = taskData,
                Status = status,
                CreatedAt = DateTime.UtcNow
            };

            await _taskCollection.InsertOneAsync(newTask);
        }

        public async Task<List<TaskDataModel>> GetTasksByUsernameAsync(string username)
        {
            var filter = Builders<TaskDataModel>.Filter.Eq(t => t.Username, username);
            var sort = Builders<TaskDataModel>.Sort.Ascending(t => t.CreatedAt); // Sort by creation time (FIFO)

            return await _taskCollection.Find(filter).Sort(sort).ToListAsync();
        }

        public async Task UpdateTaskStatusAsync(string taskId, TaskDataStatus status)
        {
            var filter = Builders<TaskDataModel>.Filter.Eq(t => t.Id, taskId);
            var update = Builders<TaskDataModel>.Update.Set(t => t.Status, status);
            await _taskCollection.UpdateOneAsync(filter, update);
        }

        public async Task DeleteAllTasksAsync()
        {
            // Delete all documents in the tasks collection
            await _taskCollection.DeleteManyAsync(FilterDefinition<TaskDataModel>.Empty);
        }

    }
}
