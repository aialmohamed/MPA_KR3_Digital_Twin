using System.Collections.Generic;
using System.Threading.Tasks;
using DigitalTwin.Data;
using DigitalTwin.Models;

namespace DigitalTwin.Services
{
    public interface ITaskService
    {
        Task AddTaskAsync(string username, TaskData taskData, TaskDataStatus status = TaskDataStatus.Idle);
        Task<List<TaskDataModel>> GetTasksByUsernameAsync(string username);
        Task UpdateTaskStatusAsync(string taskId, TaskDataStatus  status);
        Task DeleteAllTasksAsync();
    }
}
