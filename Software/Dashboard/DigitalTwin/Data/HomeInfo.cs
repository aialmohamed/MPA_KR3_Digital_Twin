
using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace DigitalTwin.Data;


public class HomeInfo
{

    public string Os_Name { get; } = RuntimeInformation.OSDescription;
    public string IsDockerInstalled  { get; private set; }
    public string IsAsyncUAInstalled { get; private set; }

    public string IsDigitalTwinImagePresent { get; private set; }
 
    public HomeInfo()
    {
        IsDockerInstalled = CheckDockerInstallation();
        IsAsyncUAInstalled = CheckAsyncUAInstallation();
        IsDigitalTwinImagePresent = CheckDockerImage("kr3r540_digital_twin");

    }
        private string CheckDockerInstallation()
        {
            return CheckCommand("docker", "--version", "Docker");
        }

        private string CheckAsyncUAInstallation()
        {
            return CheckCommand("python", "-m asyncua", "AsyncUA");
        }
                private string CheckDockerImage(string imageName)
        {
            if (IsDockerInstalled.StartsWith("Docker is not installed"))
            {
                return $"Cannot check Docker image '{imageName}' because Docker is not installed.";
            }

            string arguments = $"images -q {imageName}";
            string result = CheckCommand("docker", arguments, $"Docker image '{imageName}'");
            return string.IsNullOrWhiteSpace(result) 
                ? $"Docker image '{imageName}' does not exist." 
                : $"Docker image '{imageName}' exists.";
        }

           private string CheckCommand(string command, string arguments, string softwareName)
        {
            try
            {
                using (var process = new Process())
                {
                    process.StartInfo.FileName = command;
                    process.StartInfo.Arguments = arguments;
                    process.StartInfo.RedirectStandardOutput = true;
                    process.StartInfo.RedirectStandardError = true;
                    process.StartInfo.UseShellExecute = false;
                    process.StartInfo.CreateNoWindow = true;

                    process.Start();
                    string output = process.StandardOutput.ReadToEnd();
                    string errorOutput = process.StandardError.ReadToEnd();
                    process.WaitForExit();

                    if (process.ExitCode == 0)
                    {
                        return $"{softwareName} is installed: {output.Trim()}";
                    }
                    else
                    {
                        return $"{softwareName} is not installed: {errorOutput.Trim()}";
                    }
                }
            }
            catch (Exception ex)
            {
                // If the command fails, assume the software is not installed
                return $"{softwareName} is not installed: {ex.Message}";
            }
        }
}