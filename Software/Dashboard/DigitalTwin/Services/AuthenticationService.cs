using System;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using DigitalTwin.Database;
using DigitalTwin.Models;

using MongoDB.Driver;

namespace DigitalTwin.Services
{

    
    public interface IAuthenticationService
    {
        Task<bool> AuthenticateUserAsync(string username, string password);
        Task RegisterUserAsync(string username, string password);
    }
    
    public class AuthenticationService : IAuthenticationService
    {
        private readonly IMongoCollection<User> _userCollection;
        public AuthenticationService(MongoDBConnection mongoConnection, string databaseName = "DigitalTwinDB")
        {
            var database = mongoConnection.GetClient().GetDatabase(databaseName);
            _userCollection = database.GetCollection<User>("users");
        }

        public async Task<bool> AuthenticateUserAsync(string username, string password)
        {
            var hashedPassword = HashPassword(password);
            var filter = Builders<User>.Filter.Eq(u => u.Username, username) & Builders<User>.Filter.Eq(u => u.Password,hashedPassword);
            var user = await _userCollection.Find(filter).FirstOrDefaultAsync();

            return user != null;
        }

        public async Task RegisterUserAsync(string username, string password)
        {

            var existingUser = await _userCollection.Find(u => u.Username == username).FirstOrDefaultAsync();
            if (existingUser != null)
            {
                throw new Exception("Username already exists.");
            }
            var hashedPassword = HashPassword(password);

            var newUser = new User
            {
                Username = username,
                Password = hashedPassword
            };
            await _userCollection.InsertOneAsync(newUser);
        }


        private string HashPassword(string password)
        {
            using var sha256 = SHA256.Create();
            var bytes = Encoding.UTF8.GetBytes(password);
            var hash = sha256.ComputeHash(bytes);
            return Convert.ToBase64String(hash);
        }
    }
}