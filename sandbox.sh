#!/bin/bash
# Launch the  dev sandbox

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$SCRIPT_DIR/docker/docker-compose.dev.yml"
IMAGE_NAME="gbptrajectoryplanner"

case "${1:-}" in
    build)
        echo "Building sandbox image..."
        docker compose -f "$COMPOSE_FILE" build
        ;;
    stop)
        echo "Stopping sandbox..."
        docker compose -f "$COMPOSE_FILE" down
        ;;
    clean)
        echo "Removing sandbox containers and volumes..."
        docker compose -f "$COMPOSE_FILE" down -v
        ;;
    *)
        echo "Starting sandbox..."
        # Build if image doesn't exist yet
        if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
            echo "Image not found, building first..."
            docker compose -f "$COMPOSE_FILE" build
        fi
        docker compose -f "$COMPOSE_FILE" up -d
        docker exec -it gbptrajectoryplanner-sandbox tmux new-session -A -s dev
        ;;
esac
