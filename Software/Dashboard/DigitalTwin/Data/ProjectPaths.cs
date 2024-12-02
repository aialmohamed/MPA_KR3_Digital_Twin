


using System.IO;
using Org.BouncyCastle.Bcpg;

namespace DigitalTwin.Data;


public class ProjectPaths
{
    private readonly string _rootPath;
    private readonly string _dockerFilePath;

    public string RootPath => _rootPath; // Read-only property
    public string DockerFilePath => _dockerFilePath; // Read-only property

    public ProjectPaths()
    {
        _rootPath = Path.GetFullPath(Path.Combine(Directory.GetCurrentDirectory(), @"..\..\..\..\..\.."));
        _dockerFilePath = Path.Combine(_rootPath, "Dockerfile");
    }
}
