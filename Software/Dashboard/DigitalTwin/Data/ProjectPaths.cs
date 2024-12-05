


using System;
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
            if (OperatingSystem.IsWindows())
            {
                // Path calculation for Windows
                _rootPath = Path.GetFullPath(Path.Combine(Directory.GetCurrentDirectory(), @"..\..\..\..\..\.."));
            }
            else if (OperatingSystem.IsLinux() || OperatingSystem.IsMacOS())
            {
                // Path calculation for Linux/Mac
                _rootPath = Path.GetFullPath(Path.Combine(Directory.GetCurrentDirectory(), @"../../../"));
            }
            else
            {
                throw new NotSupportedException("Unsupported operating system.");
            }

        _dockerFilePath = Path.Combine(_rootPath, "Dockerfile");
    }
}
