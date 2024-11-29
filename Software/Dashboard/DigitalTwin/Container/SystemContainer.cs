

using System;
using System.Diagnostics;
using System.IO;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using Avalonia.Logging;
using DigitalTwin.Data;
using Opc.Ua;
using Opc.Ua.Client;
using Org.BouncyCastle.Bcpg;

namespace DigitalTwin.Container;


public class SystemContainer
{
    private readonly Func<ProjectPaths> _GetProjectPaths;
    private readonly string _dockerFilePath;
    public string DockerFilePath => _dockerFilePath;

    public SystemContainer(Func<ProjectPaths> GetProjectPaths)
    {
        _GetProjectPaths = GetProjectPaths;

    }
    public async Task EnsureDockerImageExistsAsync(string imageName , string dockerFilePath )
    {
        try
        {
            // Check if the Docker image exists
            bool imageExists = await CheckDockerImageExists(imageName);
            if (imageExists)
            {
                Console.WriteLine($"Docker image '{imageName}' already exists.");
                return;
            }

            Console.WriteLine($"Docker image '{imageName}' does not exist. Building image...");

            // Build the Docker image
            var process = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = "docker",
                    Arguments = $"build -t {imageName} {dockerFilePath}",
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    UseShellExecute = false,
                    CreateNoWindow = true
                }
            };

            process.Start();

            string output = await process.StandardOutput.ReadToEndAsync();
            string error = await process.StandardError.ReadToEndAsync();

            await process.WaitForExitAsync();

            if (process.ExitCode == 0)
            {
                Console.WriteLine($"Docker image '{imageName}' built successfully.");
                Console.WriteLine(output);
            }
            else
            {
                Console.WriteLine($"Error building Docker image '{imageName}':");
                Console.WriteLine(error);
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"An error occurred while ensuring Docker image exists: {ex.Message}");
        }
    }


    public async Task<bool> CheckDockerImageExists(string imageName)
    {
        try
        {
            using var process = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = "docker",
                    Arguments = "images --format \"{{.Repository}}:{{.Tag}}\"",
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    UseShellExecute = false,
                    CreateNoWindow = true
                }
            };

            process.Start();

            Task<string> outputTask = process.StandardOutput.ReadToEndAsync();
            Task<string> errorTask = process.StandardError.ReadToEndAsync();
            Task delayTask = Task.Delay(5000); // Timeout after 5 seconds

            // Wait for either the tasks to complete or the delay to expire
            var completedTask = await Task.WhenAny(Task.WhenAll(outputTask, errorTask), delayTask);

            if (completedTask == delayTask)
            {
                throw new TimeoutException("Docker command timed out.");
            }

            // Ensure the process has exited before checking exit code
            process.WaitForExit();

            if (process.ExitCode != 0)
            {
                string errorOutput = await errorTask;
                Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error : {errorTask.Result}");
                return false;
            }

            string output = await outputTask;
            return output.Contains(imageName);
        }
        catch (Exception ex)
        {
            Logger.TryGet(LogEventLevel.Error, "DigitalTwin")?.Log(this, $"Error : {ex.Message}");
            return false;
        }
    }
    public async Task LaunchDockerContainerInBackgroundAsync()
    {
        try
        {
            // Select the appropriate Docker command based on the platform
            string command;

            if (OperatingSystem.IsWindows())
            {
                command = "run -d -p 4840:4840 " +
                        "-v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix " +
                        "-v /run/desktop/mnt/host/wslg:/mnt/wslg " +
                        "-e DISPLAY=:0 " +
                        "-e WAYLAND_DISPLAY=wayland-0 " +
                        "-e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir " +
                        "-e PULSE_SERVER=/mnt/wslg/PulseServer " +
                        "--name kr3r540_digital_twin " +
                        "--gpus all ibo311/kr3r540_digital_twin:v1.0";
            }
            else if (OperatingSystem.IsLinux())
            {
                command = "run -d --rm --gpus all " +
                        "--net=host " +
                        "-e NVIDIA_VISIBLE_DEVICES=all " +
                        "-e NVIDIA_DRIVER_CAPABILITIES=all " +
                        "-e DISPLAY=$DISPLAY " +
                        "-v /tmp/.X11-unix:/tmp/.X11-unix " +
                        "--name kr3r540_digital_twin " +
                        "--memory-reservation=1g ibo311/kr3r540_digital_twin:v1.0";
            }
            else
            {
                throw new NotSupportedException("Unsupported operating system.");
            }

            // Start the Docker process
            var process = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = "docker", // Run Docker directly
                    Arguments = command,
                    RedirectStandardOutput = true, // Capture output
                    RedirectStandardError = true, // Capture error
                    UseShellExecute = false,      // Run in background without terminal
                    CreateNoWindow = true         // Prevent terminal from opening
                }
            };

            process.Start();

            // Capture the output and error streams
            string output = await process.StandardOutput.ReadToEndAsync();
            string error = await process.StandardError.ReadToEndAsync();

            await process.WaitForExitAsync();

            if (process.ExitCode == 0)
            {
                Console.WriteLine("Docker container launched successfully in background:");
                Console.WriteLine(output);
            }
            else
            {
                Console.WriteLine("Error launching Docker container in background:");
                Console.WriteLine(error);
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"An error occurred while launching Docker in the background: {ex.Message}");
        }
    }
    public async Task<bool> WaitForOpcUaServerAsync(string endpointUrl, int timeoutSeconds = 100)
    {
        int elapsedSeconds = 0;

        while (elapsedSeconds < timeoutSeconds)
        {
            try
            {
                // Create OPC UA client configuration
                var config = new ApplicationConfiguration
                {
                    ApplicationName = "OpcUaClient",
                    ApplicationType = ApplicationType.Client,
                    SecurityConfiguration = new SecurityConfiguration
                    {
                        ApplicationCertificate = new CertificateIdentifier(),
                        AutoAcceptUntrustedCertificates = true
                    },
                    ClientConfiguration = new ClientConfiguration { DefaultSessionTimeout = 60000 }
                };

                await config.Validate(ApplicationType.Client);

                // Discover endpoint
                var endpoint = CoreClientUtils.SelectEndpoint(endpointUrl, useSecurity: false);

                // Attempt to create a session
                using var session = await Session.Create(config, new ConfiguredEndpoint(null, endpoint), false, "OpcUaClient", 60000, null, null);

                if (session.Connected)
                {
                    Console.WriteLine("OPC UA server is ready.");
                    return true;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Server not ready yet: {ex.Message}");
            }

            await Task.Delay(1000); // Wait for 1 second before retrying
            elapsedSeconds++;
        }

        Console.WriteLine("Timeout waiting for OPC UA server to become ready.");
        return false;
    }

    public async Task StopAndRemoveDockerContainerAsync(string containerName)
    {
        try
        {
            // Stop the container
            var stopProcess = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = "docker",
                    Arguments = $"stop {containerName}",
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    UseShellExecute = false,
                    CreateNoWindow = true
                }
            };

            stopProcess.Start();
            string stopOutput = await stopProcess.StandardOutput.ReadToEndAsync();
            string stopError = await stopProcess.StandardError.ReadToEndAsync();
            await stopProcess.WaitForExitAsync();

            if (stopProcess.ExitCode != 0)
            {
                Console.WriteLine($"Error stopping Docker container '{containerName}':");
                Console.WriteLine(stopError);
            }
            else
            {
                Console.WriteLine($"Docker container '{containerName}' stopped successfully.");
                Console.WriteLine(stopOutput);
            }

            // Remove the container
            var removeProcess = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = "docker",
                    Arguments = $"rm {containerName}",
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    UseShellExecute = false,
                    CreateNoWindow = true
                }
            };

            removeProcess.Start();
            string removeOutput = await removeProcess.StandardOutput.ReadToEndAsync();
            string removeError = await removeProcess.StandardError.ReadToEndAsync();
            await removeProcess.WaitForExitAsync();

            if (removeProcess.ExitCode != 0)
            {
                Console.WriteLine($"Error removing Docker container '{containerName}':");
                Console.WriteLine(removeError);
            }
            else
            {
                Console.WriteLine($"Docker container '{containerName}' removed successfully.");
                Console.WriteLine(removeOutput);
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"An error occurred while stopping and removing the Docker container: {ex.Message}");
        }
    }




}