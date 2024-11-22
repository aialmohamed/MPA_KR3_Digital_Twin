using System.Collections.Generic;
using System.Threading.Tasks;
using MongoDB.Driver;
using DigitalTwin.Models;
using DigitalTwin.Database;

namespace DigitalTwin.Services
{
    public class UserService
    {
        private readonly IMongoCollection<User> _users;

        public UserService(MongoDBConnection mongoConnection, string databaseName = "DigitalTwinDB")
        {
            var database = mongoConnection.GetClient().GetDatabase(databaseName);
            _users = database.GetCollection<User>("users"); // "users" is the collection name
        }

        // CREATE
        public async Task CreateUserAsync(User user)
        {
            await _users.InsertOneAsync(user);
        }

        // READ - Get All Users
        public async Task<List<User>> GetUsersAsync()
        {
            return await _users.Find(_ => true).ToListAsync(); // Fetch all users
        }

        // READ - Get User By ID
        public async Task<User> GetUserByIdAsync(string id)
        {
            return await _users.Find(user => user.Id == id).FirstOrDefaultAsync();
        }

        // UPDATE - Update User By ID
        public async Task UpdateUserAsync(string id, User updatedUser)
        {
            await _users.ReplaceOneAsync(user => user.Id == id, updatedUser);
        }

        // DELETE - Delete User By ID
        public async Task DeleteUserAsync(string id)
        {
            await _users.DeleteOneAsync(user => user.Id == id);
        }
    }
}
