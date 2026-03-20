#!/usr/bin/env bash
# Setup NVIDIA Container Toolkit for Docker GPU passthrough in WSL2.
# Run this on your WSL2 host (NOT inside Docker).
#
# Usage: bash scripts/setup-nvidia-docker.sh
#
# Prerequisites:
#   - WSL2 with NVIDIA GPU driver installed on Windows
#   - Docker Desktop or Docker Engine running in WSL2
#   - nvidia-smi works in WSL2

set -euo pipefail

echo "=== NVIDIA Docker GPU Passthrough Setup ==="
echo ""

# 1. Verify we're in WSL2, not inside Docker
if [ -f /.dockerenv ]; then
    echo "ERROR: Run this on your WSL2 host, NOT inside Docker."
    exit 1
fi

# 2. Verify nvidia-smi works
if ! command -v nvidia-smi &>/dev/null; then
    echo "ERROR: nvidia-smi not found. Install NVIDIA GPU driver on Windows first."
    exit 1
fi
echo "[1/5] GPU detected:"
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
echo ""

# 3. Install NVIDIA Container Toolkit
echo "[2/5] Installing nvidia-container-toolkit..."
if ! dpkg -l nvidia-container-toolkit &>/dev/null; then
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
        | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg 2>/dev/null

    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
        | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
        | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null

    sudo apt-get update -qq
    sudo apt-get install -y -qq nvidia-container-toolkit
    echo "  Installed."
else
    echo "  Already installed."
fi
echo ""

# 4. Configure Docker runtime
echo "[3/5] Configuring Docker runtime for NVIDIA..."
sudo nvidia-ctk runtime configure --runtime=docker 2>/dev/null
echo ""

# 5. Restart Docker
echo "[4/5] Restarting Docker..."
if command -v systemctl &>/dev/null && systemctl is-active docker &>/dev/null; then
    sudo systemctl restart docker
    echo "  Docker restarted via systemctl."
elif pgrep -x dockerd &>/dev/null; then
    sudo pkill dockerd && sleep 2 && sudo dockerd &>/dev/null &
    echo "  Docker daemon restarted."
else
    echo "  WARNING: Could not restart Docker automatically."
    echo "  If using Docker Desktop, restart it from the Windows system tray."
    echo "  Press Enter after restarting Docker Desktop..."
    read -r
fi
echo ""

# 6. Verify GPU is visible in Docker
echo "[5/5] Verifying GPU access in Docker..."
if docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null; then
    echo ""
    echo "=== SUCCESS ==="
    echo "GPU is accessible from Docker containers."
    echo ""
    echo "Next steps:"
    echo "  cd $(dirname "$0")/../docker"
    echo "  docker compose -f docker-compose.dev.yml down"
    echo "  docker compose -f docker-compose.dev.yml up -d"
    echo "  docker exec -it gbptrajectoryplanner-sandbox bash"
    echo ""
    echo "Inside the container, verify with: nvidia-smi"
else
    echo ""
    echo "=== PARTIAL SUCCESS ==="
    echo "Toolkit installed but GPU test container failed."
    echo "This is common on first run. Try:"
    echo "  1. Restart Docker Desktop (Windows system tray)"
    echo "  2. Re-run this script"
    echo ""
    echo "If using Docker Desktop, ensure:"
    echo "  Settings → Resources → WSL Integration → enable for your distro"
fi
