<p align="center"><img src="./payload/runtime/docker/setup/app/elesim_setup/web/icon.svg" width="180" alt="EleSim logo"></p>

<h1 align="center">EleSim</h1>

<p align="center">https://doi.org/10.1109/LRA.2026.3663818</p>

<br>

## 1. How to Start

### 🧙 Installation Wizard

Run the following command.

   ```bash
   curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/install.sh | bash
   ```

### 🐳 Tailscale with Docker Desktop

While using Docker Desktop, you need to generate a emulated Tailscale IP for DDS communication.

   ```bash
   elesim-tailscale login
   ```

### 🐘 Connection Manager

Once the setup is complete with `elesim-connections`, you can start right away by using `elesim-up` next time.

   ```bash
   elesim-connections
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
elesim-up                  # Run EleSim

elesim-down                # Stop EleSim
elesim-down --purge        # Stop EleSim and remove the connection manager

elesim-update              # Update EleSim

elesim-connections         # Manage the connections between apps and run EleSim

elesim-logs                # Check the logs
elesim-status              # Check the execution info on this computer

elesim-tailscale login     # Log in to Tailscale with native Docker
elesim-tailscale update    # Update the emulated Tailscale

elesim-uninstall           # Remove EleSim
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
