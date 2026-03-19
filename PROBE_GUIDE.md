  ---
  Step 1: Windows Host — Install usbipd-win

  Install https://github.com/dorssel/usbipd-win/releases (if not already done). Then attach your probe to WSL each time you
  plug it in:

  # In Windows PowerShell (Admin) - list USB devices
  usbipd list

  # Bind the device (one-time, needs admin)
  usbipd bind --busid <BUSID>

  # Attach to WSL (each session)
  usbipd attach --wsl --busid <BUSID>

  ---
  Step 2: WSL — Install udev Rules

  Run these inside WSL (not in the container):

  # Download probe-rs udev rules
  curl -fsSL https://probe.rs/files/69-probe-rs.rules | sudo tee /etc/udev/rules.d/69-probe-rs.rules

  # Create plugdev group and add your WSL user
  sudo groupadd --system plugdev
  sudo usermod -a -G plugdev $USER

  # Reload udev
  sudo udevadm control --reload
  sudo udevadm trigger

  ---
  Step 3: WSL — Ensure udev Runs at Boot

  # Check if running
  service udev status

  # If not running, configure it to auto-start
  sudo tee -a /etc/wsl.conf <<'EOF'
  [boot]
  command="service udev start"
  EOF

  Then restart WSL: wsl --shutdown from PowerShell, then reopen.

  ---
  Step 4: Dockerfile — Add esp User to plugdev Group

  The device files in /dev are owned by WSL's GIDs. The esp user in the container needs to be in plugdev (GID typically 46
  on Debian). Update your Dockerfile:

● Update(.devcontainer/Dockerfile)
  ⎿  Added 2 lines
      24      adduser --disabled-password --gecos "" ${CONTAINER_USER}
      25      usermod -aG dialout ${CONTAINER_USER}
      26      echo "${CONTAINER_USER}    ALL=(ALL:ALL) ALL" >>/etc/sudoers
      27 +    groupadd --system --gid 46 plugdev || true
      28 +    usermod -aG plugdev ${CONTAINER_USER}
      29  EOF
      30  USER ${CONTAINER_USER}
      31  WORKDIR /home/${CONTAINER_USER}

● ---
  Step 5: Verify the GID Matches

  After WSL setup, check the actual GID of plugdev in WSL:

  # In WSL
  getent group plugdev
  # Output: plugdev:x:46:youruser  ← note the GID

  If it's not 46, update the --gid 46 in the Dockerfile to match.

  ---
  Step 6: Rebuild the Devcontainer

  In VS Code: Ctrl+Shift+P → "Dev Containers: Rebuild Container"

  ---
  Verify It Works

  Inside the devcontainer:
  probe-rs list

  You should see your probe listed. If you get a permission error, check:
  ls -la /dev/bus/usb/*/*   # check device permissions
  id                         # verify esp user is in plugdev group