<p align="center"><img src="./payload/runtime/docker/setup/app/elesim_setup/setup_web/icon.svg" width="180" alt="EleSim logo"></p>

<h1 align="center">EleSim</h1>

<p align="center">https://doi.org/10.1109/LRA.2026.3663818</p>

<br>

## 1. How to Start

### 🧙 Setup Wizard

Run the following command.

   ```bash
   curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/install.sh | bash
   ```

### 🐳 Tailscale with Docker Desktop

While using Docker Desktop, you need to generate a emulated Tailscale IP for DDS communication.

   ```bash
   ./elesim tailscale login
   ```

### 🐘 Connection Manager

Run `./elesim connections` to configure your system ID and launch EleSim for the first time or after changing your environment.

For future sessions, simply use `./elesim up <system-id>` to start it.

   ```bash
   ./elesim connections
   ```

<br>

## 2. Apps and Commands

### 💻 Applications

| Name | Role |
| --- | --- |
| Pilot | Calculator |
| UI | Control Panel |
| Sim | Simulation on [Genesis World](https://github.com/Genesis-Embodied-AI/genesis-world) |
| Robot | On-board Software for Jetson |

DDS is used by default, while video is sent via WebRTC.

### ⌨️ Commands

```bash
./elesim connections         # Manage topology and system registrations

./elesim up <system-id>      # Start a registered system
./elesim down <system-id>    # Stop a registered system
./elesim logs <system-id>    # Check its logs
./elesim info <system-id>    # Show its status
./elesim remove <system-id>  # Remove the system registration

./elesim update              # Update/build/publish EleSim

./elesim tailscale login     # Log in to the Docker Desktop Tailscale sidecar
./elesim tailscale status    # Show sanitized Tailscale status

./elesim uninstall           # Remove EleSim
```

<br>

## 3. Documents

[Setup](docs/setup.md)  
[Deployment](docs/deployment.md)  
[Configuration](docs/configuration.md)  
[Architecture](docs/architecture.md)  
[DDS Contracts](docs/dds_contracts.md)  
[Dev Status](docs/status.md)  
[Research](docs/research.md)
