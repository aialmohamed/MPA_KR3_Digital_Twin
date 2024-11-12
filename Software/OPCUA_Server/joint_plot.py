import matplotlib.pyplot as plt
import re
import pandas as pd

# Sample log data
log_data = """
[ros2_control_node-2] [INFO] [1731423226.014058826] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.028308035] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.042705656] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.057370295] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.071760231] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.085400498] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.099662223] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.113919219] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.128257281] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.142510578] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.157596370] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.171111182] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.185401935] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.199670327] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.213913820] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.228211205] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.242692120] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.256840838] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.271388603] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.285347047] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.299707007] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.314041429] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.330353848] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.347490779] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.358236759] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.372476491] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.385365530] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.404423324] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.416421257] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.431403770] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.442485991] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.457043555] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.471132031] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.485366883] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.499674196] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.513968744] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.528395595] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.542752080] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.557045073] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.571216048] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.585383317] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.599665722] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.613912601] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.628286094] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.642567631] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.656967840] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.671131970] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.685379304] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.699742171] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.713913742] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.728316392] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.742485735] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.757427652] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.771277164] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.785383213] [Kr3r540Interface]: Converted command_data: {A1 -23.962, A2 -97.633, A3 102.624, A4 -78.920, A5 -24.447, A6 -12.140}
[ros2_control_node-2] [INFO] [1731423226.799648099] [Kr3r540Interface]: Converted command_data: {A1 -20.884, A2 -97.375, A3 100.948, A4 -77.484, A5 -21.005, A6 -13.742}
[ros2_control_node-2] [INFO] [1731423226.813911012] [Kr3r540Interface]: Converted command_data: {A1 -17.808, A2 -97.118, A3 99.275, A4 -76.050, A5 -17.567, A6 -15.343}
[ros2_control_node-2] [INFO] [1731423226.828257898] [Kr3r540Interface]: Converted command_data: {A1 -14.716, A2 -96.859, A3 97.591, A4 -74.608, A5 -14.110, A6 -16.953}
[ros2_control_node-2] [INFO] [1731423226.842514529] [Kr3r540Interface]: Converted command_data: {A1 -11.640, A2 -96.602, A3 95.917, A4 -73.173, A5 -10.672, A6 -18.553}
[ros2_control_node-2] [INFO] [1731423226.856810099] [Kr3r540Interface]: Converted command_data: {A1 -8.558, A2 -96.344, A3 94.240, A4 -71.736, A5 -7.226, A6 -20.157}
[ros2_control_node-2] [INFO] [1731423226.871360701] [Kr3r540Interface]: Converted command_data: {A1 -5.479, A2 -96.086, A3 92.564, A4 -70.300, A5 -3.784, A6 -21.760}
[ros2_control_node-2] [INFO] [1731423226.885353097] [Kr3r540Interface]: Converted command_data: {A1 -2.400, A2 -95.828, A3 90.888, A4 -68.864, A5 -0.341, A6 -23.363}
[ros2_control_node-2] [INFO] [1731423226.899642085] [Kr3r540Interface]: Converted command_data: {A1 0.681, A2 -95.571, A3 89.211, A4 -67.427, A5 3.103, A6 -24.966}
[ros2_control_node-2] [INFO] [1731423226.914111952] [Kr3r540Interface]: Converted command_data: {A1 3.787, A2 -95.311, A3 87.520, A4 -65.978, A5 6.575, A6 -26.583}
[ros2_control_node-2] [INFO] [1731423226.928234937] [Kr3r540Interface]: Converted command_data: {A1 6.847, A2 -95.055, A3 85.855, A4 -64.551, A5 9.996, A6 -28.176}
[ros2_control_node-2] [INFO] [1731423226.943306228] [Kr3r540Interface]: Converted command_data: {A1 10.096, A2 -94.783, A3 84.086, A4 -63.035, A5 13.629, A6 -29.867}
[ros2_control_node-2] [INFO] [1731423226.956803557] [Kr3r540Interface]: Converted command_data: {A1 13.008, A2 -94.539, A3 82.501, A4 -61.678, A5 16.884, A6 -31.383}
[ros2_control_node-2] [INFO] [1731423226.971107757] [Kr3r540Interface]: Converted command_data: {A1 16.095, A2 -94.281, A3 80.821, A4 -60.238, A5 20.335, A6 -32.989}
[ros2_control_node-2] [INFO] [1731423226.985364999] [Kr3r540Interface]: Converted command_data: {A1 19.169, A2 -94.023, A3 79.147, A4 -58.804, A5 23.772, A6 -34.590}
[ros2_control_node-2] [INFO] [1731423226.999651084] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.014135122] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.028428714] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.042527166] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.056800773] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.071072386] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.085379050] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.099723272] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.114295439] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.128228918] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.143218673] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.156808120] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.171457387] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.185342826] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.199738459] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.214064302] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.228427871] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.242520054] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.256930513] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.271207580] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.285365575] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.299652781] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.313947902] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.328378142] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.342605304] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.356811783] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.371152503] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.385374651] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.399657367] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.413953071] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.428245759] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.442740169] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.456828003] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.471147542] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.485416732] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.499948562] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.513926050] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.528242835] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.542501581] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.557194195] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.571052772] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.585374987] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.600022782] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.613954097] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.628199585] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.642497540] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.656905325] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.671096481] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.685417036] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.699812387] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.714059967] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.728519757] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.743179727] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.757366903] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.771217730] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.785615674] [Kr3r540Interface]: Converted command_data: {A1 19.172, A2 -94.023, A3 79.146, A4 -58.803, A5 23.776, A6 -34.591}
[ros2_control_node-2] [INFO] [1731423227.799644138] [Kr3r540Interface]: Converted command_data: {A1 19.475, A2 -93.998, A3 78.981, A4 -58.662, A5 24.114, A6 -34.749}
[ros2_control_node-2] [INFO] [1731423227.814067463] [Kr3r540Interface]: Converted command_data: {A1 19.786, A2 -93.972, A3 78.812, A4 -58.517, A5 24.462, A6 -34.910}
[ros2_control_node-2] [INFO] [1731423227.828214715] [Kr3r540Interface]: Converted command_data: {A1 20.091, A2 -93.946, A3 78.646, A4 -58.374, A5 24.803, A6 -35.069}
[ros2_control_node-2] [INFO] [1731423227.842545658] [Kr3r540Interface]: Converted command_data: {A1 20.400, A2 -93.920, A3 78.478, A4 -58.230, A5 25.149, A6 -35.230}
[ros2_control_node-2] [INFO] [1731423227.856786270] [Kr3r540Interface]: Converted command_data: {A1 20.707, A2 -93.895, A3 78.310, A4 -58.087, A5 25.492, A6 -35.390}
[ros2_control_node-2] [INFO] [1731423227.871077018] [Kr3r540Interface]: Converted command_data: {A1 21.015, A2 -93.869, A3 78.143, A4 -57.943, A5 25.836, A6 -35.550}
[ros2_control_node-2] [INFO] [1731423227.885552130] [Kr3r540Interface]: Converted command_data: {A1 21.328, A2 -93.843, A3 77.973, A4 -57.797, A5 26.186, A6 -35.713}
[ros2_control_node-2] [INFO] [1731423227.901031824] [Kr3r540Interface]: Converted command_data: {A1 21.646, A2 -93.816, A3 77.799, A4 -57.649, A5 26.542, A6 -35.879}
[ros2_control_node-2] [INFO] [1731423227.914169247] [Kr3r540Interface]: Converted command_data: {A1 21.944, A2 -93.791, A3 77.637, A4 -57.510, A5 26.875, A6 -36.034}
[ros2_control_node-2] [INFO] [1731423227.928344199] [Kr3r540Interface]: Converted command_data: {A1 22.250, A2 -93.765, A3 77.470, A4 -57.367, A5 27.217, A6 -36.193}
[ros2_control_node-2] [INFO] [1731423227.942490516] [Kr3r540Interface]: Converted command_data: {A1 22.556, A2 -93.740, A3 77.304, A4 -57.225, A5 27.558, A6 -36.352}
[ros2_control_node-2] [INFO] [1731423227.956909681] [Kr3r540Interface]: Converted command_data: {A1 22.866, A2 -93.714, A3 77.135, A4 -57.080, A5 27.905, A6 -36.514}
[ros2_control_node-2] [INFO] [1731423227.971314195] [Kr3r540Interface]: Converted command_data: {A1 23.177, A2 -93.688, A3 76.966, A4 -56.935, A5 28.253, A6 -36.676}
[ros2_control_node-2] [INFO] [1731423227.985375723] [Kr3r540Interface]: Converted command_data: {A1 23.480, A2 -93.663, A3 76.801, A4 -56.793, A5 28.592, A6 -36.833}
[ros2_control_node-2] [INFO] [1731423228.001409943] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.014160603] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.028290739] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.042517296] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.056752448] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.071077967] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.085378638] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.099681885] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.114367485] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.128218861] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.142538586] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.156829177] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.171145772] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.185370476] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.199686691] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.213952951] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.228244280] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.243541124] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.257142753] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.271064627] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.285353533] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.301669067] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.313933887] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.328309778] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.342506543] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.356955497] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.371295360] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.385923374] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.399615963] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.413884055] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.428503319] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.442568009] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.456798054] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.472336637] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.485353780] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.499647793] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.513930913] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.528232967] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.542567557] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.557518473] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.571242500] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.585359342] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.599708982] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.614254825] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.628261714] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.642879836] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.657533645] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.671098222] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.685447905] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.701165230] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.714066317] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.728379096] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.742534235] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.756795946] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.771109707] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.785627712] [Kr3r540Interface]: Converted command_data: {A1 23.486, A2 -93.662, A3 76.798, A4 -56.791, A5 28.598, A6 -36.836}
[ros2_control_node-2] [INFO] [1731423228.799785502] [Kr3r540Interface]: Converted command_data: {A1 23.516, A2 -93.659, A3 76.781, A4 -56.777, A5 28.632, A6 -36.852}
[ros2_control_node-2] [INFO] [1731423228.814372121] [Kr3r540Interface]: Converted command_data: {A1 23.547, A2 -93.657, A3 76.764, A4 -56.762, A5 28.667, A6 -36.868}
[ros2_control_node-2] [INFO] [1731423228.828214173] [Kr3r540Interface]: Converted command_data: {A1 23.578, A2 -93.654, A3 76.748, A4 -56.748, A5 28.701, A6 -36.884}
[ros2_control_node-2] [INFO] [1731423228.842699788] [Kr3r540Interface]: Converted command_data: {A1 23.609, A2 -93.652, A3 76.731, A4 -56.734, A5 28.736, A6 -36.900}
[ros2_control_node-2] [INFO] [1731423228.856759232] [Kr3r540Interface]: Converted command_data: {A1 23.639, A2 -93.649, A3 76.714, A4 -56.719, A5 28.769, A6 -36.916}
[ros2_control_node-2] [INFO] [1731423228.871391536] [Kr3r540Interface]: Converted command_data: {A1 23.671, A2 -93.647, A3 76.697, A4 -56.705, A5 28.805, A6 -36.932}
[ros2_control_node-2] [INFO] [1731423228.885390243] [Kr3r540Interface]: Converted command_data: {A1 23.701, A2 -93.644, A3 76.681, A4 -56.691, A5 28.838, A6 -36.948}
[ros2_control_node-2] [INFO] [1731423228.899834501] [Kr3r540Interface]: Converted command_data: {A1 23.732, A2 -93.641, A3 76.664, A4 -56.676, A5 28.873, A6 -36.964}
[ros2_control_node-2] [INFO] [1731423228.913941565] [Kr3r540Interface]: Converted command_data: {A1 23.762, A2 -93.639, A3 76.647, A4 -56.662, A5 28.907, A6 -36.980}
[ros2_control_node-2] [INFO] [1731423228.928429096] [Kr3r540Interface]: Converted command_data: {A1 23.794, A2 -93.636, A3 76.630, A4 -56.647, A5 28.942, A6 -36.996}
[ros2_control_node-2] [INFO] [1731423228.942992156] [Kr3r540Interface]: Converted command_data: {A1 23.825, A2 -93.634, A3 76.613, A4 -56.633, A5 28.977, A6 -37.013}
[ros2_control_node-2] [INFO] [1731423228.957200385] [Kr3r540Interface]: Converted command_data: {A1 23.856, A2 -93.631, A3 76.597, A4 -56.618, A5 29.012, A6 -37.029}
[ros2_control_node-2] [INFO] [1731423228.971483183] [Kr3r540Interface]: Converted command_data: {A1 23.886, A2 -93.629, A3 76.580, A4 -56.604, A5 29.046, A6 -37.045}
[ros2_control_node-2] [INFO] [1731423228.985348956] [Kr3r540Interface]: Converted command_data: {A1 23.916, A2 -93.626, A3 76.563, A4 -56.590, A5 29.079, A6 -37.060}
[ros2_control_node-2] [INFO] [1731423228.999645303] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.015027459] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.028227796] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.042858556] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.056796994] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.071243781] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.085339435] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.099640571] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.113987702] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.128304278] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.142507031] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.157205950] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.171619979] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.185609933] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.199645305] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.213933992] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.228235850] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.242515374] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.257022256] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.271109319] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.285373681] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.299654068] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}
[ros2_control_node-2] [INFO] [1731423229.313971457] [Kr3r540Interface]: Converted command_data: {A1 23.917, A2 -93.626, A3 76.563, A4 -56.590, A5 29.080, A6 -37.061}

"""

# Regular expression to extract timestamp and joint values
# Regular expression to extract timestamp and joint values
pattern = r'\[(\d+\.\d+)\] \[Kr3r540Interface\]: Converted command_data: \{A1 ([\d.-]+), A2 ([\d.-]+), A3 ([\d.-]+), A4 ([\d.-]+), A5 ([\d.-]+), A6 ([\d.-]+)\}'

# Extract timestamp and joint positions
data = []
for match in re.finditer(pattern, log_data):
    timestamp = float(match.group(1))
    positions = [float(match.group(i)) for i in range(2, 8)]  # Extract A1 to A6
    data.append([timestamp] + positions)

# Define columns for the DataFrame
columns = ['timestamp', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Create the DataFrame
df = pd.DataFrame(data, columns=columns)

# Plot each joint's movement against time
for joint in columns[1:]:  # Exclude 'timestamp'
    plt.figure()
    plt.plot(df['timestamp'].to_numpy(), df[joint].to_numpy(), marker='o', label=joint)  # Convert Series to numpy array
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title(f'Movement of {joint} over time')
    plt.legend()
    plt.show()